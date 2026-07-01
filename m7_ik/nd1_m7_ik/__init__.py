"""
nd1_m7_ik — ND1 M7 모듈 로봇공학 IK 패키지
==========================================
피지컬 AI 전문가 과정(ND1) M7 모듈 산출물.
3-DOF 평면 로봇의 FK / 해석적 야코비안 / 조작성 / 수치 IK 제공.

Dockerfile 호환 공개 API:
    from nd1_m7_ik import (
        RobotArm3DOF,
        jacobian_analytical_3dof,
        manipulability,
        numerical_ik,
    )
"""
from .arm import RobotArm3DOF, dh_matrix
from .jacobian import jacobian_analytical_3dof, jacobian_numerical, manipulability
from .ik import numerical_ik

__version__ = "1.1.0"

__all__ = [
    "RobotArm3DOF",
    "dh_matrix",
    "jacobian_analytical_3dof",
    "jacobian_numerical",
    "manipulability",
    "numerical_ik",
    "__version__",
]
