"""설치 검증용 스모크 테스트. `python -m tests.smoke` 또는 직접 실행."""
import numpy as np

from nd1_m7_ik import (
    RobotArm3DOF,
    jacobian_analytical_3dof,
    manipulability,
    numerical_ik,
)
from nd1_m7_ik.jacobian import jacobian_numerical


def main():
    robot = RobotArm3DOF()

    # 1) FK 검증
    assert np.allclose(robot.ee_position([0, 0, 0]), [0.8, 0.0], atol=1e-9)
    assert np.allclose(robot.ee_position([np.pi / 2, 0, 0]), [0.0, 0.8], atol=1e-9)

    # 2) 해석 야코비안 == 수치 야코비안
    th = np.array([0.3, -0.5, 0.8])
    Ja = jacobian_analytical_3dof(th)
    Jn = jacobian_numerical(robot, th)
    assert Ja.shape == (2, 3)
    assert np.allclose(Ja, Jn, atol=1e-5), f"\n{Ja}\n{Jn}"

    # 3) 조작성: J 입력 / thetas 입력 둘 다 동일
    assert np.isclose(manipulability(Ja), manipulability(th))

    # 4) 수치 IK 수렴 (PBL 합격 기준 1e-6) — 항상 (theta, history) 반환
    for target in [(0.6, 0.2), (0.4, 0.4), (0.0, 0.7)]:
        theta, hist = numerical_ik(robot, target, return_history=True)
        err = np.linalg.norm(robot.ee_position(theta) - np.array(target))
        assert err < 1e-6, f"target={target} err={err:.2e}"
        print(f"[IK] target={target} step={len(hist):>4} err={hist[-1]:.2e} OK")

    print("ALL SMOKE TESTS PASSED")


if __name__ == "__main__":
    main()
