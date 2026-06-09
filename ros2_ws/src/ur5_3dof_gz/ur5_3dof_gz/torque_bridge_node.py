"""
Bridges /torque_command (JointState from ur5_pid_node) to the topics
expected by the ros2_control effort / position controllers in Gazebo.

  /torque_command  →  /arm_effort_controller/commands    (joints 1-3, effort)
                  →  /wrist_position_controller/commands (joints 4-6, pos=0)

A periodic timer republishes the most recently cached values at 50 Hz:
  - arm effort defaults to [0, 0, 0] until a torque_command arrives, so
    the "uncontrolled" launch (no PID) works out of the box.
  - wrist position is always [0, 0, 0] (wrists frozen by design).
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TorqueBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("torque_bridge")

        self._effort_pub = self.create_publisher(
            Float64MultiArray, "/arm_effort_controller/commands", 10)
        self._wrist_pub = self.create_publisher(
            Float64MultiArray, "/wrist_position_controller/commands", 10)

        self._latest_effort = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            JointState, "/torque_command", self._torque_cb, 10)

        # Republish at 50 Hz — controllers expect a steady stream
        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            "Torque bridge ready — publishing zero effort + zero wrist position "
            "until a /torque_command arrives.")

    def _tick(self) -> None:
        with self._lock:
            effort = list(self._latest_effort)

        eff_msg = Float64MultiArray()
        eff_msg.data = effort
        self._effort_pub.publish(eff_msg)

        wrist_msg = Float64MultiArray()
        wrist_msg.data = [0.0, 0.0, 0.0]
        self._wrist_pub.publish(wrist_msg)

    def _torque_cb(self, msg: JointState) -> None:
        if len(msg.effort) < 3:
            return
        with self._lock:
            self._latest_effort = [float(msg.effort[0]),
                                   float(msg.effort[1]),
                                   float(msg.effort[2])]


def main() -> None:
    rclpy.init()
    node = TorqueBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
