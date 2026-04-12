#!/usr/bin/env python3
"""
pub_loop_closure_retrigger (stub)

Re-publishes a loop-closure retrigger signal on demand. This is a
placeholder: the original ROS1 implementation was never checked in to
master, and the `closure_retrigger.launch.py` launch file has always
referenced it. This stub exists so that the launch file resolves to a
real executable and `ros2 launch scan2shape_launch closure_retrigger.launch.py`
no longer fails with "executable not found".

The stub node subscribes to `~/trigger` (std_msgs/Empty) and republishes
every message it receives on `~/retrigger` (std_msgs/Empty). Replace
this body with the full retrigger logic when you port it.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class LoopClosureRetrigger(Node):
    def __init__(self):
        super().__init__("loop_closure_retrigger")
        self.get_logger().warn(
            "pub_loop_closure_retrigger.py is a stub — the original "
            "implementation was not checked in. This node will forward "
            "messages from ~/trigger to ~/retrigger, nothing more."
        )
        self._pub = self.create_publisher(Empty, "~/retrigger", 10)
        self._sub = self.create_subscription(
            Empty, "~/trigger", self._on_trigger, 10
        )

    def _on_trigger(self, _msg: Empty) -> None:
        self._pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureRetrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
