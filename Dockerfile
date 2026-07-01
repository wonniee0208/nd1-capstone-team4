# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 통합 실습 환경
#  베이스: ROS2 Humble + noVNC (브라우저 데스크탑) — Windows 학생용
#  기준: Ubuntu 22.04 / ROS2 Humble / Gazebo Fortress(ign gazebo v6) / Python 3.10
# ════════════════════════════════════════════════════════════════
FROM tiryoh/ros2-desktop-vnc:humble

USER root
SHELL ["/bin/bash", "-c"]

# ── 1. Gazebo Classic 제거 (Fortress와 충돌 방지) ──────────────────
RUN apt-get update && apt-get remove -y "*gazebo*" && apt-get autoremove -y

# ── 2. Gazebo Fortress 명시 설치 (명령어: ign gazebo) ──────────────
RUN apt-get install -y lsb-release wget gnupg && \
    wget https://packages.osrfoundation.org/gazebo.gpg \
      -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y ros-humble-ros-gz

# ── 3. Nav2 + TurtleBot4 (M8 연계) ────────────────────────────────
#    ⚠️ turtlebot3-gazebo(Classic 전용)는 Fortress와 충돌 → 설치 금지
RUN apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation

# ── 4. Foxglove + 분석 라이브러리 (M13 연계) ──────────────────────
RUN apt-get install -y ros-humble-foxglove-bridge python3-pip && \
    pip3 install --no-cache-dir matplotlib numpy scipy rosbags

# ── 5. Node A(LLM) 의존성 + 테스트 도구 ───────────────────────────
RUN pip3 install --no-cache-dir groq pydantic python-dotenv pytest

# ── 6. M7 IK 코어 (Node C가 nd1_m7_ik 임포트) ─────────────────────
COPY m7_ik /opt/nd1/m7_ik
RUN pip3 install --no-cache-dir /opt/nd1/m7_ik && \
    python3 -c "from nd1_m7_ik import RobotArm3DOF, jacobian_analytical_3dof, manipulability; print('nd1_m7_ik OK')"

# ── 7. 사용자 / 워크스페이스 ──────────────────────────────────────
RUN id -u ubuntu &>/dev/null || \
    (useradd -m -s /bin/bash ubuntu && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers)
RUN mkdir -p /home/ubuntu/ros2_ws/src && chown -R ubuntu:ubuntu /home/ubuntu

# ── 8. ★ Windows/WSL2 안정 기동 — 소프트웨어 렌더링 기본값 ─────────
#    WSL 가상 GPU의 OpenGL 미구현으로 인한 Ogre GL3Plus 크래시를 원천 차단.
#    (가속이 필요한 네이티브 Linux 학생은 compose override로 이 값을 끄면 됨)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV QT_X11_NO_MITSHM=1

# Node A 기본 LLM 모델 (2026-06-17 llama-3.x 계열 deprecated → gpt-oss 권장)
ENV GROQ_MODEL=openai/gpt-oss-20b

# ⚠️ USER ubuntu 라인 없음 — 베이스 시작 스크립트(supervisord)가 root여야 VNC 정상 구동
WORKDIR /home/ubuntu/ros2_ws
