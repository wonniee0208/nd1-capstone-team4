#!/usr/bin/env python3
# Node A — 자연어 명령 해석
# 역할: /llm_command(String) → /mission(String, JSON)
#
# 구현 원칙
# - Coordinator/Node C와 맞춘 공통 계약은 유지한다.
# - 기획서 표현(A구역/B구역/electronic_parts_box/전자부품 상자)은
#   내부 공통 이름(Shelf_1/Workbench/parts_box)으로 정규화한다.
# - GROQ_API_KEY가 없거나 LLM 응답이 이상하면 키워드 폴백 파서로 동작한다.

import json
import os
import re
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_ITEMS = ["parts_box", "tool_box", "sensor_box"]
VALID_SOURCES = ["Shelf_1", "Shelf_2", "Shelf_3"]
VALID_TARGETS = ["Worker", "Workbench"]


# source는 Coordinator 계약상 선반 위치만 허용한다.
# 기획서의 A/B/C 구역명은 창고 작업 구역 별칭으로 받아들인다.
SOURCE_ALIASES = {
    "1번선반": "Shelf_1",
    "1번": "Shelf_1",
    "선반1": "Shelf_1",
    "shelf1": "Shelf_1",
    "shelf_1": "Shelf_1",
    "a구역": "Shelf_1",
    "a존": "Shelf_1",
    "azone": "Shelf_1",
    "aarea": "Shelf_1",

    "2번선반": "Shelf_2",
    "2번": "Shelf_2",
    "선반2": "Shelf_2",
    "shelf2": "Shelf_2",
    "shelf_2": "Shelf_2",
    "b출발구역": "Shelf_2",
    "b선반": "Shelf_2",

    "3번선반": "Shelf_3",
    "3번": "Shelf_3",
    "선반3": "Shelf_3",
    "shelf3": "Shelf_3",
    "shelf_3": "Shelf_3",
    "c구역": "Shelf_3",
    "c존": "Shelf_3",
}


# target은 Coordinator 계약상 Worker 또는 Workbench만 허용한다.
# 기획서의 B구역은 배치 지점으로 보고 Workbench로 정규화한다.
TARGET_ALIASES = {
    "작업자": "Worker",
    "워커": "Worker",
    "worker": "Worker",

    "작업대": "Workbench",
    "워크벤치": "Workbench",
    "workbench": "Workbench",
    "b구역": "Workbench",
    "b존": "Workbench",
    "bzone": "Workbench",
    "barea": "Workbench",
    "도착지": "Workbench",
    "배치구역": "Workbench",
}


# 기획서의 electronic_parts_box는 Node C가 parts_box를 전자부품 상자로 해석하므로
# 여기서는 Coordinator가 받을 수 있는 parts_box로 정규화한다.
ITEM_ALIASES = {
    "전자부품상자": "parts_box",
    "전자부품박스": "parts_box",
    "전자부품": "parts_box",
    "electronicpartsbox": "parts_box",
    "electronic_parts_box": "parts_box",
    "부품박스": "parts_box",
    "부품상자": "parts_box",
    "부품": "parts_box",
    "partsbox": "parts_box",
    "parts_box": "parts_box",
    "박스": "parts_box",
    "상자": "parts_box",
    "box": "parts_box",
    "electronicsbox": "parts_box",

    "공구박스": "tool_box",
    "공구상자": "tool_box",
    "공구": "tool_box",
    "toolbox": "tool_box",
    "tool_box": "tool_box",

    "센서박스": "sensor_box",
    "센서상자": "sensor_box",
    "센서": "sensor_box",
    "sensorbox": "sensor_box",
    "sensor_box": "sensor_box",
}


SYSTEM_PROMPT = """
너는 창고 피킹 보조 로봇의 명령 파서다.
한국어 자연어 명령을 아래 JSON 형식으로만 변환한다.

반드시 이 JSON 키만 사용한다:
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Workbench"
}

내부 위치 이름:
- 1번 선반, A구역: Shelf_1
- 2번 선반: Shelf_2
- 3번 선반: Shelf_3
- 작업자: Worker
- 작업대, B구역: Workbench

내부 물품 이름:
- 전자부품 상자, 전자부품 박스, 부품 박스: parts_box
- 공구 박스: tool_box
- 센서 박스: sensor_box

주의:
- electronic_parts_box라는 이름은 쓰지 말고 parts_box로 출력한다.
- fragile 필드는 출력하지 않는다. 전자부품 상자 안전 처리는 Node C가 담당한다.
- 설명 없이 JSON 객체 하나만 출력한다.
"""


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
        text = msg.data.strip()
        self._status(f"명령 수신: '{text}'")

        mission = self._parse_with_llm(text)
        if mission is None:
            mission = self._parse_fallback(text)

        if mission is None:
            self._status(f"명령 해석 실패: '{text}'")
            return

        mission_json = json.dumps(mission, ensure_ascii=False)
        self.pub.publish(String(data=mission_json))
        self._status(f"미션 발행: {mission_json}")

    def _parse_with_llm(self, text: str) -> Optional[Dict[str, str]]:
        if self._llm is None:
            return None

        try:
            response = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
            )

            content = response.choices[0].message.content
            raw = json.loads(content)
            mission = self._normalize_mission(raw)

            if self._valid_mission(mission):
                return mission

            self._status(f"LLM 응답 형식 오류: {raw}")
            return None

        except Exception as e:
            self._status(f"LLM 파싱 실패, 폴백 사용: {e}")
            return None

    def _parse_fallback(self, text: str) -> Optional[Dict[str, str]]:
        norm_text = self._norm(text)

        if self._contains_any(norm_text, ["정지", "멈춰", "멈추", "스톱", "stop", "cancel"]):
            return {"task": "stop"}

        source = self._find_alias(norm_text, SOURCE_ALIASES)
        target = self._find_alias(norm_text, TARGET_ALIASES)
        item = self._find_alias(norm_text, ITEM_ALIASES)

        if source and target and item:
            return {
                "task": "pick_and_deliver",
                "item": item,
                "source": source,
                "target": target,
            }

        self._status(f"폴백 해석 실패: source={source}, target={target}, item={item}")
        return None

    def _normalize_mission(self, raw: Any) -> Optional[Dict[str, str]]:
        if not isinstance(raw, dict):
            return None

        task = str(raw.get("task", "")).strip()
        norm_task = self._norm(task)

        if norm_task in ("stop", "cancel", "정지", "멈춰", "멈추"):
            return {"task": "stop"}

        # LLM이 pick_and_place, delivery 등으로 살짝 바꿔도 내부 계약으로 보정한다.
        if norm_task not in (
            "pickanddeliver",
            "pickandplace",
            "deliver",
            "delivery",
            "movebox",
            "운반",
            "옮기기",
        ):
            task = "pick_and_deliver"

        item = self._normalize_field(raw.get("item"), ITEM_ALIASES)
        source = self._normalize_field(
            raw.get("source", raw.get("from", raw.get("pickup", raw.get("start")))),
            SOURCE_ALIASES,
        )
        target = self._normalize_field(
            raw.get("target", raw.get("to", raw.get("destination", raw.get("place")))),
            TARGET_ALIASES,
        )

        if not (item and source and target):
            return None

        return {
            "task": "pick_and_deliver",
            "item": item,
            "source": source,
            "target": target,
        }

    def _normalize_field(self, value: Any, aliases: Dict[str, str]) -> Optional[str]:
        if value is None:
            return None

        norm_value = self._norm(str(value))

        # 이미 내부 이름으로 온 경우도 alias 테이블에서 처리한다.
        found = self._find_alias(norm_value, aliases)
        if found:
            return found

        # 별칭에 등록되지 않은 내부 이름을 마지막으로 직접 검증한다.
        raw = str(value).strip()
        if raw in VALID_ITEMS or raw in VALID_SOURCES or raw in VALID_TARGETS:
            return raw

        return None

    @staticmethod
    def _norm(text: str) -> str:
        return re.sub(r"[\s_\-:/,.;()\[\]{}]+", "", str(text)).lower()

    @staticmethod
    def _contains_any(norm_text: str, keywords) -> bool:
        return any(NodeALLM._norm(k) in norm_text for k in keywords)

    def _find_alias(self, norm_text: str, aliases: Dict[str, str]) -> Optional[str]:
        # 긴 별칭부터 확인해야 '공구박스'가 일반 '박스'보다 먼저 잡힌다.
        for alias, internal in sorted(aliases.items(), key=lambda kv: len(self._norm(kv[0])), reverse=True):
            if self._norm(alias) in norm_text:
                return internal
        return None

    @staticmethod
    def _valid_mission(mission: Optional[Dict[str, str]]) -> bool:
        if not isinstance(mission, dict):
            return False

        task = mission.get("task")
        if task == "stop":
            return True

        if task != "pick_and_deliver":
            return False

        return (
            mission.get("item") in VALID_ITEMS
            and mission.get("source") in VALID_SOURCES
            and mission.get("target") in VALID_TARGETS
        )

    def _init_groq(self):
        key = os.environ.get("GROQ_API_KEY", "").strip()
        if not key or key.startswith("your_"):
            return None

        try:
            from groq import Groq
            return Groq(api_key=key)
        except Exception as e:
            self.get_logger().warn(f"Groq 초기화 실패, 폴백 사용: {e}")
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
