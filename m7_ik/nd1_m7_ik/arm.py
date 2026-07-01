"""
nd1_m7_ik.arm
=============
3-DOF 평면 로봇 팔의 순기구학(FK).

링크 길이 기본값: L1=L2=0.3m, L3=0.2m  (M7 모듈 표준)
좌표계: 베이스 원점(0,0), z축 회전만 사용하는 평면(planar) 로봇.

FK 수식
-------
x = L1·cos(θ1) + L2·cos(θ1+θ2) + L3·cos(θ1+θ2+θ3)
y = L1·sin(θ1) + L2·sin(θ1+θ2) + L3·sin(θ1+θ2+θ3)

검증값:
    fk([0,0,0])          -> EE (0.8, 0.0)
    fk([pi/2, 0, 0])     -> EE (0.0, 0.8)
"""
from __future__ import annotations

import numpy as np

__all__ = ["dh_matrix", "RobotArm3DOF"]


def dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """표준 DH 4x4 동차변환행렬.

    평면 로봇에서는 d=0, alpha=0, a=링크길이, theta=관절각으로 사용한다.
    T = Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,      d],
            [0.0,     0.0,      0.0,    1.0],
        ]
    )


class RobotArm3DOF:
    """3-DOF 평면 로봇 팔.

    Parameters
    ----------
    links : sequence of float, optional
        링크 길이 [L1, L2, L3]. 기본 [0.3, 0.3, 0.2].

    Examples
    --------
    >>> robot = RobotArm3DOF()
    >>> positions, T = robot.fk([0.0, 0.0, 0.0])
    >>> round(T[0, 3], 3), round(T[1, 3], 3)
    (0.8, 0.0)
    """

    #: 기본 링크 길이 (클래스 상수 — M7 표준)
    LINKS = (0.3, 0.3, 0.2)

    def __init__(self, links=None):
        self.links = tuple(float(x) for x in (links if links is not None else self.LINKS))
        if len(self.links) != 3:
            raise ValueError("RobotArm3DOF는 정확히 3개의 링크 길이가 필요합니다.")

    # ------------------------------------------------------------------ #
    # 순기구학
    # ------------------------------------------------------------------ #
    def fk(self, thetas):
        """순기구학.

        Parameters
        ----------
        thetas : array-like, shape (3,)
            관절각 [θ1, θ2, θ3] (라디안).

        Returns
        -------
        positions : list[tuple[float, float]]
            [베이스, 관절1, 관절2, 말단부(EE)] 평면 좌표. 시각화용.
        T : np.ndarray, shape (4, 4)
            베이스→말단부 동차변환행렬. EE 위치는 T[:2, 3].
        """
        thetas = np.asarray(thetas, dtype=float).reshape(-1)
        if thetas.shape[0] != 3:
            raise ValueError("thetas는 길이 3이어야 합니다.")

        T = np.eye(4)
        positions = [(0.0, 0.0)]
        for link, theta in zip(self.links, thetas):
            T = T @ dh_matrix(a=link, d=0.0, alpha=0.0, theta=theta)
            positions.append((float(T[0, 3]), float(T[1, 3])))
        return positions, T

    def ee_position(self, thetas) -> np.ndarray:
        """말단부(EE)의 평면 위치 (x, y)만 빠르게 반환."""
        _, T = self.fk(thetas)
        return np.array([T[0, 3], T[1, 3]])

    def __repr__(self) -> str:  # pragma: no cover
        return f"RobotArm3DOF(links={self.links})"
