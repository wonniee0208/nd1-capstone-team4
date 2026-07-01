"""
nd1_m7_ik.jacobian
==================
3-DOF 평면 로봇의 해석적 야코비안과 Yoshikawa 조작성 지표.

해석적 야코비안 (2x3)
---------------------
    s1   = sin(θ1)
    s12  = sin(θ1+θ2)
    s123 = sin(θ1+θ2+θ3)        (c도 동일)

    J = [[-L1·s1 - L2·s12 - L3·s123,  -L2·s12 - L3·s123,  -L3·s123],
         [ L1·c1 + L2·c12 + L3·c123,   L2·c12 + L3·c123,   L3·c123]]

조작성 (Yoshikawa, 1984)
------------------------
    w = sqrt(det(J · Jᵀ))
    w → 0 : 특이점(singularity) 근접 → 역행렬 발산 위험
"""
from __future__ import annotations

import numpy as np

from .arm import RobotArm3DOF

__all__ = ["jacobian_analytical_3dof", "jacobian_numerical", "manipulability"]


def jacobian_analytical_3dof(thetas, links=RobotArm3DOF.LINKS) -> np.ndarray:
    """3-DOF 평면 로봇의 해석적 야코비안 (2x3).

    Parameters
    ----------
    thetas : array-like, shape (3,)
        관절각 [θ1, θ2, θ3] (라디안).
    links : sequence of float, optional
        링크 길이 [L1, L2, L3]. 기본 [0.3, 0.3, 0.2].

    Returns
    -------
    np.ndarray, shape (2, 3)
        말단부 속도와 관절 속도의 관계 ẋ = J·θ̇.
    """
    thetas = np.asarray(thetas, dtype=float).reshape(-1)
    if thetas.shape[0] != 3:
        raise ValueError("thetas는 길이 3이어야 합니다.")
    L1, L2, L3 = (float(x) for x in links)
    t1 = thetas[0]
    t12 = thetas[0] + thetas[1]
    t123 = thetas[0] + thetas[1] + thetas[2]

    s1, c1 = np.sin(t1), np.cos(t1)
    s12, c12 = np.sin(t12), np.cos(t12)
    s123, c123 = np.sin(t123), np.cos(t123)

    J = np.array(
        [
            [-L1 * s1 - L2 * s12 - L3 * s123, -L2 * s12 - L3 * s123, -L3 * s123],
            [ L1 * c1 + L2 * c12 + L3 * c123,  L2 * c12 + L3 * c123,  L3 * c123],
        ]
    )
    return J


def jacobian_numerical(robot, thetas, eps: float = 1e-7) -> np.ndarray:
    """중앙차분 수치 야코비안 (2x3). 해석해 검증용.

    Parameters
    ----------
    robot : RobotArm3DOF
    thetas : array-like, shape (3,)
    eps : float
        섭동 크기.
    """
    thetas = np.asarray(thetas, dtype=float).reshape(-1)
    J = np.zeros((2, thetas.shape[0]))
    for i in range(thetas.shape[0]):
        tp = thetas.copy(); tp[i] += eps
        tm = thetas.copy(); tm[i] -= eps
        J[:, i] = (robot.ee_position(tp) - robot.ee_position(tm)) / (2.0 * eps)
    return J


def manipulability(J_or_thetas, links=RobotArm3DOF.LINKS) -> float:
    """Yoshikawa 조작성 지표 w = sqrt(det(J·Jᵀ)).

    편의를 위해 두 가지 입력을 모두 허용한다.

    Parameters
    ----------
    J_or_thetas : np.ndarray
        - shape (2, 3) 야코비안 행렬을 직접 넘기거나,
        - shape (3,)  관절각을 넘기면 내부에서 해석적 야코비안을 계산한다.
    links : sequence of float, optional
        thetas를 넘긴 경우에만 사용되는 링크 길이.

    Returns
    -------
    float
        조작성 지표 w (>= 0). 0에 가까울수록 특이점.
    """
    arr = np.asarray(J_or_thetas, dtype=float)
    if arr.ndim == 1:  # 관절각으로 해석
        J = jacobian_analytical_3dof(arr, links=links)
    else:
        J = arr
    w_sq = np.linalg.det(J @ J.T)
    # 수치오차로 아주 작은 음수가 나올 수 있어 0으로 클램프
    return float(np.sqrt(max(w_sq, 0.0)))
