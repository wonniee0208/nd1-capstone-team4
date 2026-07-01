#!/usr/bin/env python3

import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    UNDOCKING = auto()
    NAVIGATING = auto()
    GRASPING = auto()
    TRANSPORTING = auto()
    PLACING = auto()
    DOCKING = auto()
    DONE = auto()
    FAILED = auto()


class CoordinatorFSM(Node):
    def __init__(self):
        super().__init__("coordinator_fsm")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("auto_redock", True)

        self.sim_mode = self.get_parameter("sim_mode").value
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.auto_redock = bool(self.get_parameter("auto_redock").value)

        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.pub_nav = self.create_publisher(String, "/nav_request", 10)
        self.pub_grasp = self.create_publisher(String, "/grasp_request", 10)
        self.pub_dock = self.create_publisher(String, "/dock_request", 10)

        self.create_subscription(String, "/mission", self._on_mission, 10)
        self.create_subscription(Bool, "/nav_result", self._on_nav_result, 10)
        self.create_subscription(Bool, "/grasp_result", self._on_grasp_result, 10)
        self.create_subscription(Bool, "/dock_result", self._on_dock_result, 10)

        self.state = State.IDLE
        self.cmd = None
        self.retries = 0
        self._busy = False
        self._done = False
        self._ok = False

        hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self._status("Coordinator started")

    def _on_mission(self, msg: String):
        try:
            new_cmd = json.loads(msg.data)
        except Exception as e:
            self._status("mission json error: " + str(e))
            return

        if self.state not in (State.IDLE, State.DONE, State.FAILED):
            self._status("busy: ignore new mission")
            return

        self.cmd = new_cmd
        self.retries = 0
        self._reset_flags()
        self.state = State.PLANNING
        self._status("mission received: " + str(self.cmd.get("action")))

    def _tick(self):
        if self.state in (State.IDLE, State.DONE, State.FAILED):
            return

        if self.state == State.PLANNING:
            action = self.cmd.get("action")

            if action == "stop":
                self.state = State.DONE
                self._status("stop -> DONE")
                return

            if action in ("navigate", "pick_and_place"):
                self._enter_dock("undock", State.UNDOCKING)
                return

            self.state = State.FAILED
            self._status("unknown action -> FAILED")
            return

        if self._busy and not self._done:
            return

        if self._busy and self._done:
            self._busy = False
            self._advance(self._ok)

    def _advance(self, success: bool):
        if not success:
            if self.retries < self.max_retries:
                self.retries += 1
                self._status("retry " + str(self.retries))
                self._retry_current()
                return

            self.state = State.FAILED
            self._status("FAILED")
            return

        self.retries = 0
        action = self.cmd.get("action")

        if self.state == State.UNDOCKING:
            if action == "navigate":
                self._enter_nav(
                    self.cmd.get("place_x", 0.0),
                    self.cmd.get("place_y", 0.0),
                    State.NAVIGATING,
                )
                return

            if action == "pick_and_place":
                self._enter_nav(
                    self.cmd.get("pick_x", 0.0),
                    self.cmd.get("pick_y", 0.0),
                    State.NAVIGATING,
                )
                return

        if self.state == State.NAVIGATING:
            if action == "navigate":
                self._finish_mission()
                return

            if action == "pick_and_place":
                self._enter_grasp(
                    "grasp",
                    self.cmd.get("pick_x", 0.0),
                    self.cmd.get("pick_y", 0.0),
                    State.GRASPING,
                )
                return

        if self.state == State.GRASPING:
            self._enter_nav(
                self.cmd.get("place_x", 0.0),
                self.cmd.get("place_y", 0.0),
                State.TRANSPORTING,
            )
            return

        if self.state == State.TRANSPORTING:
            self._enter_grasp(
                "place",
                self.cmd.get("place_x", 0.0),
                self.cmd.get("place_y", 0.0),
                State.PLACING,
            )
            return

        if self.state == State.PLACING:
            self._finish_mission()
            return

        if self.state == State.DOCKING:
            self.state = State.DONE
            self._status("DONE")
            return

        self.state = State.FAILED
        self._status("transition failed")

    def _finish_mission(self):
        if self.auto_redock:
            self._status("mission complete: redock")
            self._enter_dock("dock", State.DOCKING)
        else:
            self.state = State.DONE
            self._status("DONE")

    def _retry_current(self):
        action = self.cmd.get("action")

        if self.state == State.UNDOCKING:
            self._enter_dock("undock", State.UNDOCKING)
            return

        if self.state == State.NAVIGATING:
            if action == "navigate":
                self._enter_nav(
                    self.cmd.get("place_x", 0.0),
                    self.cmd.get("place_y", 0.0),
                    State.NAVIGATING,
                )
                return

            if action == "pick_and_place":
                self._enter_nav(
                    self.cmd.get("pick_x", 0.0),
                    self.cmd.get("pick_y", 0.0),
                    State.NAVIGATING,
                )
                return

        if self.state == State.GRASPING:
            self._enter_grasp(
                "grasp",
                self.cmd.get("pick_x", 0.0),
                self.cmd.get("pick_y", 0.0),
                State.GRASPING,
            )
            return

        if self.state == State.TRANSPORTING:
            self._enter_nav(
                self.cmd.get("place_x", 0.0),
                self.cmd.get("place_y", 0.0),
                State.TRANSPORTING,
            )
            return

        if self.state == State.PLACING:
            self._enter_grasp(
                "place",
                self.cmd.get("place_x", 0.0),
                self.cmd.get("place_y", 0.0),
                State.PLACING,
            )
            return

        if self.state == State.DOCKING:
            self._enter_dock("dock", State.DOCKING)
            return

        self.state = State.FAILED
        self._status("retry failed")

    def _enter_dock(self, op: str, next_state: State):
        self.state = next_state
        self._reset_flags()
        self._busy = True
        self._status(op + " request [" + next_state.name + "]")

        if self.sim_mode:
            self._simulate(True, 1.0)
            return

        self.pub_dock.publish(String(data=json.dumps({"op": op})))

    def _enter_nav(self, x, y, next_state: State):
        x = float(x)
        y = float(y)

        self.state = next_state
        self._reset_flags()
        self._busy = True
        self._status("nav request (" + str(x) + ", " + str(y) + ") [" + next_state.name + "]")

        if self.sim_mode:
            self._simulate(True, 2.0)
            return

        data = {
            "x": x,
            "y": y,
            "yaw": float(self.cmd.get("yaw", 0.0)),
        }
        self.pub_nav.publish(String(data=json.dumps(data)))

    def _enter_grasp(self, op: str, x, y, next_state: State):
        x = float(x)
        y = float(y)

        self.state = next_state
        self._reset_flags()
        self._busy = True
        self._status(op + " request (" + str(x) + ", " + str(y) + ") [" + next_state.name + "]")

        if self.sim_mode:
            self._simulate(True, 1.5)
            return

        data = {
            "op": op,
            "x": x,
            "y": y,
        }
        self.pub_grasp.publish(String(data=json.dumps(data)))

    def _on_dock_result(self, msg: Bool):
        if self._busy and self.state in (State.UNDOCKING, State.DOCKING):
            self._finish(bool(msg.data))

    def _on_nav_result(self, msg: Bool):
        if self._busy and self.state in (State.NAVIGATING, State.TRANSPORTING):
            self._finish(bool(msg.data))

    def _on_grasp_result(self, msg: Bool):
        if self._busy and self.state in (State.GRASPING, State.PLACING):
            self._finish(bool(msg.data))

    def _simulate(self, success: bool, secs: float):
        holder = {}

        def done():
            holder["timer"].cancel()
            self._finish(success)

        holder["timer"] = self.create_timer(secs, done)

    def _finish(self, ok: bool):
        self._ok = ok
        self._done = True

    def _reset_flags(self):
        self._busy = False
        self._done = False
        self._ok = False

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data="[FSM:" + self.state.name + "] " + text))


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()