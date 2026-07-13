"""
Gripper mimic relay node.

Subscribes to /joint_states and forwards gripper_left_joint position to
/gripper_right_mimic_controller/commands (std_msgs/Float64MultiArray) so that
Gazebo Fortress physically moves the right finger in sync with the left.

Gazebo Fortress does not enforce URDF <mimic> constraints in physics; the
ign_ros2_control native mimic params produce a "_mimic" suffix in the published
joint name, which breaks MoveIt's model lookup.  This relay node provides the
same bilateral behaviour without any of those side-effects:

  - gripper_left_joint  <- GripperActionController (commanded by grasp_server)
  - gripper_right_joint <- ForwardCommandController <- THIS NODE

The relay is sim-only and should be launched alongside
gazebo_headless.launch.py / gazebo.launch.py.  On real hardware the jaw is
mechanically linked so only the left joint is commanded; do NOT start this node
in real-robot bringup.

Published topic: /gripper_right_mimic_controller/commands
                 std_msgs/msg/Float64MultiArray  [position in metres, 0..0.04]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GripperMimicRelay(Node):
    """Mirror gripper_left_joint position to the right-finger command topic."""

    def __init__(self) -> None:
        super().__init__("gripper_mimic_relay")

        self._left_joint = "gripper_left_joint"
        self._left_idx: int | None = None  # cached /joint_states index
        self._last_position: float = -1.0  # sentinel — force first publish

        # Match joint_state_broadcaster QoS: RELIABLE + VOLATILE
        js_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_right_mimic_controller/commands",
            10,
        )

        self._js_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._js_callback,
            js_qos,
        )

        self.get_logger().info(
            f"GripperMimicRelay started — mirroring {self._left_joint}"
            " -> /gripper_right_mimic_controller/commands"
        )

    def _js_callback(self, msg: JointState) -> None:
        """Forward gripper_left_joint position to the right-finger topic."""
        # Joint ordering from a given broadcaster is stable, so resolve the
        # index once and only re-scan if the cached slot no longer matches.
        idx = self._left_idx
        if idx is None or idx >= len(msg.name) or msg.name[idx] != self._left_joint:
            try:
                idx = msg.name.index(self._left_joint)
            except ValueError:
                self._left_idx = None
                return
            self._left_idx = idx

        if not msg.position or idx >= len(msg.position):
            return

        pos = float(msg.position[idx])
        if abs(pos - self._last_position) < 1e-6:
            return

        self._last_position = pos
        cmd = Float64MultiArray()
        cmd.data = [pos]
        self._cmd_pub.publish(cmd)
        self.get_logger().debug(
            f"Forwarded gripper_right position: {pos:.5f} m"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperMimicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
