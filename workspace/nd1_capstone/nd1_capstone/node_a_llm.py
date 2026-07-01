#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node A — 자연어 명령 해석 [학생 구현]
#  역할: /llm_command(String) → groq LLM 파싱(+폴백) → /mission(String, JSON)
#
#  제공(인프라): 데이터 계약(RobotCommand), pub/sub, groq 클라이언트, main
#  구현(TODO):  ① _parse_with_llm  ② _parse_fallback
#
#  토픽 계약(고정):
#    In  /llm_command (std_msgs/String) — 사용자 자연어
#    Out /mission     (std_msgs/String) — RobotCommand JSON 1건
#  표준 좌표: A(1.5,0.5) B(2.5,-1.0) C(0.5,2.0)  ※ 변경 금지
# ════════════════════════════════════════════════════════════════
import json
import os
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydantic import BaseModel, Field

ZONES = {"A": (1.5, 0.5), "B": (2.5, -1.0), "C": (0.5, 2.0)}


class ActionType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NAVIGATE = "navigate"
    STOP = "stop"


class RobotCommand(BaseModel):
    """LLM/폴백이 생성하는 구조화 명령 (이 스키마를 그대로 /mission 으로 발행)."""
    action: ActionType
    object: str = ""
    pick_x: float = 0.0
    pick_y: float = 0.0
    place_x: float = 0.0
    place_y: float = 0.0
    yaw: float = Field(default=0.0)


SYSTEM_PROMPT = """너는 로봇 명령 파서다. 한국어 명령을 RobotCommand JSON으로만 변환한다.
구역 좌표: A=(1.5,0.5), B=(2.5,-1.0), C=(0.5,2.0).
스키마: {"action":"pick_and_place|navigate|stop","object":"","pick_x":0,"pick_y":0,"place_x":0,"place_y":0,"yaw":0}
설명 없이 JSON 객체 하나만 출력."""


class NodeALLM(Node):
    def __init__(self):
        super().__init__("node_a_llm")
        self.pub = self.create_publisher(String, "/mission", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/llm_command", self._on_command, 10)
        self._llm = self._init_groq()
        self._model = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")
        self._status(f"Node A 시작 — LLM={'ON' if self._llm else 'OFF(폴백)'}")

    def _on_command(self, msg: String):
        text = msg.data
        self._status(f"명령 수신: '{text}'")
        cmd = self._parse_with_llm(text) or self._parse_fallback(text)
        self.pub.publish(String(data=cmd.model_dump_json()))
        self._status(f"미션 발행: {cmd.action.value} pick=({cmd.pick_x},{cmd.pick_y}) "
                     f"place=({cmd.place_x},{cmd.place_y})")

    # ── ① TODO: groq LLM 파싱 ─────────────────────────────────────
    def _parse_with_llm(self, text: str):
        """groq로 자연어 → RobotCommand. 실패/미가용이면 None 반환(→ 폴백).
        힌트:
          - self._llm 이 None 이면 곧장 return None
          - self._llm.chat.completions.create(model=self._model, temperature=0,
                response_format={"type":"json_object"},
                messages=[{"role":"system","content":SYSTEM_PROMPT},
                          {"role":"user","content":text}])
          - 응답 JSON을 RobotCommand(**json.loads(...)) 로 검증해 반환
          - 예외는 try/except로 잡고 return None (폴백이 받도록)
        """
        # TODO: 위 힌트대로 구현
        return None

    # ── ② TODO: 키워드 폴백 파서 (groq 없거나 실패 시) ────────────
    def _parse_fallback(self, text: str) -> RobotCommand:
        """규칙 기반 파서. groq 차단 상황에서도 데모가 돌아가게 하는 안전망."""
        t = text.upper().replace(" ", "")

        # 1) 정지 명령은 최우선 처리
        stop_keywords = ["정지", "멈춰", "멈추", "스톱", "STOP"]
        if any(k in t for k in stop_keywords):
            return RobotCommand(action=ActionType.STOP)

        # 2) 문장에 등장한 구역을 순서대로 추출
        zones = []
        for i, ch in enumerate(t):
            if ch in ZONES:
                # A구역, B구역, A, B 같은 표현 모두 허용
                if ch not in zones:
                    zones.append(ch)

        # 3) 물체 이동 / 운반 / 배치 명령
        move_keywords = ["옮", "이동", "놓", "배치", "운반", "가져", "집어", "PICK", "PLACE", "MOVE", "FROM", "TO"]
        has_move = any(k in t for k in move_keywords)

        if has_move and len(zones) >= 2:
            pick_zone = zones[0]
            place_zone = zones[1]
            px, py = ZONES[pick_zone]
            gx, gy = ZONES[place_zone]
            return RobotCommand(
                action=ActionType.PICK_AND_PLACE,
                object=self._extract_object(text),
                pick_x=px,
                pick_y=py,
                place_x=gx,
                place_y=gy,
                yaw=0.0,
            )

        # 4) 구역 하나만 있으면 단순 이동
        if len(zones) == 1:
            gx, gy = ZONES[zones[0]]
            return RobotCommand(
                action=ActionType.NAVIGATE,
                object="",
                pick_x=0.0,
                pick_y=0.0,
                place_x=gx,
                place_y=gy,
                yaw=0.0,
            )

        # 5) 해석 불가 시 안전하게 정지
        return RobotCommand(action=ActionType.STOP)

    @staticmethod
    def _extract_object(text: str) -> str:
        # (선택) "~를/을" 앞 단어를 object로. 미구현 시 "박스" 고정도 무방.
        return "박스"

    def _init_groq(self):
        key = os.environ.get("GROQ_API_KEY", "").strip()
        if not key or key.startswith("your_"):
            return None
        try:
            from groq import Groq
            return Groq(api_key=key)
        except Exception:
            return None

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[A] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeALLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
