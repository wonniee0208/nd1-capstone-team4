"""
nd1_m7_ik.ik
============
감쇠 최소자승법(Damped Least Squares, DLS) 기반 수치 역기구학.

반복식
------
    e  = p_target - p_current                 (위치 오차, 2x1)
    Δθ = Jᵀ (J Jᵀ + λ²I)⁻¹ e
    θ ← θ + Δθ

λ(감쇠 계수)는 특이점 근처에서 (J Jᵀ)가 비가역이 되어 발산하는 것을 막는다.

반환 규약 (캡스톤 Node C 계약)
------------------------------
    기본은 ``theta`` (관절각 ndarray, shape (3,))만 반환한다.
    Node C가 ``list(numerical_ik(robot, (x, y)))`` 로 호출해
    ``[θ1, θ2, θ3]`` 평면 리스트를 기대하기 때문이다.

    오차 히스토리가 필요하면 ``return_history=True`` 로 호출하면
    ``(theta, history)`` 2-튜플을 반환한다. (M7 실습/수렴 곡선용)
"""
from __future__ import annotations

import numpy as np

from .arm import RobotArm3DOF
from .jacobian import jacobian_analytical_3dof

__all__ = ["numerical_ik"]


def numerical_ik(
    robot: RobotArm3DOF,
    target,
    theta_init=None,
    tol: float = 1e-6,
    max_iter: int = 1000,
    lam: float = 1e-4,
    return_history: bool = False,
):
    """수치 역기구학 (DLS).

    Parameters
    ----------
    robot : RobotArm3DOF
        대상 로봇.
    target : array-like
        목표 위치. (x, y) 또는 (x, y, z) 모두 허용(z는 무시, 평면 로봇).
    theta_init : array-like, shape (3,), optional
        초기 관절각. 기본 [0, 0, 0].
    tol : float
        수렴 허용 오차(m).
    max_iter : int
        최대 반복 횟수.
    lam : float
        감쇠 계수 λ.
    return_history : bool, default False
        True이면 ``(theta, history)`` 반환. 기본은 ``theta`` 단독 반환.

    Returns
    -------
    theta : np.ndarray, shape (3,)
        수렴된(또는 마지막) 관절각. (기본 반환값)
    (theta, history) : tuple
        return_history=True일 때. history는 스텝별 오차 노름 리스트.
    """
    target = np.asarray(target, dtype=float).reshape(-1)[:2]
    theta = (
        np.zeros(3)
        if theta_init is None
        else np.asarray(theta_init, dtype=float).reshape(-1).copy()
    )
    if theta.shape[0] != 3:
        raise ValueError("theta_init은 길이 3이어야 합니다.")

    I2 = np.eye(2)
    history = []
    for _ in range(max_iter):
        p = robot.ee_position(theta)
        e = target - p
        err = float(np.linalg.norm(e))
        history.append(err)
        if err < tol:
            break
        J = jacobian_analytical_3dof(theta, links=robot.links)
        J_dls = J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * I2)
        theta = theta + J_dls @ e

    if return_history:
        return theta, history
    return theta
