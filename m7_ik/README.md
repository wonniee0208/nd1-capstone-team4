# nd1_m7_ik (v1.1.0)

ND1 피지컬 AI 전문가 과정 **M7 모듈** 산출물 — 3-DOF 평면 로봇의
순기구학(FK) · 해석적 야코비안 · 조작성 · 수치 역기구학(IK) 패키지.

## Dockerfile 호환 공개 API

```python
from nd1_m7_ik import (
    RobotArm3DOF,            # 3-DOF 평면 로봇 (LINKS=[0.3,0.3,0.2]), .fk() / .ee_position()
    jacobian_analytical_3dof,# 해석적 야코비안 (2x3)
    manipulability,          # Yoshikawa 조작성 w = sqrt(det(J·Jᵀ))
    numerical_ik,            # DLS 수치 IK
)
```

## 설치 (Dockerfile 6단계)

```dockerfile
COPY m7_ik /opt/nd1/m7_ik
RUN pip3 install /opt/nd1/m7_ik
```

## 빠른 사용 예시

```python
import numpy as np
from nd1_m7_ik import RobotArm3DOF, jacobian_analytical_3dof, manipulability, numerical_ik

robot = RobotArm3DOF()                       # links 기본 [0.3, 0.3, 0.2]
print(robot.ee_position([0, 0, 0]))          # -> [0.8, 0.0]

J = jacobian_analytical_3dof([0.3, -0.5, 0.8])   # (2,3)
print(manipulability(J))                          # 조작성 지표
print(manipulability([0.3, -0.5, 0.8]))           # thetas 직접 입력도 허용

theta, hist = numerical_ik(robot, (0.6, 0.2))   # 항상 (theta, history) 반환
print("수렴 step:", len(hist), "최종 오차:", hist[-1])  # < 1e-6
```

## 검증

```bash
pip install .
python -m tests.smoke    # FK/Jacobian/Manipulability/IK 일괄 검증
```

## 구조

```
m7_ik/
├── setup.py             # 메타데이터(구형 pip 호환)
├── pyproject.toml       # 빌드 백엔드 선언
├── README.md
├── nd1_m7_ik/
│   ├── __init__.py      # 공개 API export + __version__
│   ├── arm.py           # RobotArm3DOF, dh_matrix
│   ├── jacobian.py      # jacobian_analytical_3dof, jacobian_numerical, manipulability
│   └── ik.py            # numerical_ik (DLS)
└── tests/
    └── smoke.py
```
