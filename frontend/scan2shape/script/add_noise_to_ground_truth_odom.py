#!/usr/bin/env python3
"""
add_noise_to_ground_truth_odom (stub)

Adds Gaussian noise to a ground-truth odometry stream for simulation /
benchmarking. This is a placeholder: the original ROS1 implementation
was never checked in to master, and the `sim_perturb_odom.launch.py`
launch file has always referenced it. This stub exists so that the
launch file resolves to a real executable and
`ros2 launch scan2shape_launch sim_perturb_odom.launch.py` no longer
fails with "executable not found".

The stub node subscribes to `~/odom_gt` (nav_msgs/Odometry), adds
zero-mean Gaussian noise with the configured sigma to the linear
position, and republishes on `~/odom_noisy`. Replace this body with
the full noise model when you port it — the original ROS1 version
perturbed both pose and velocity with covariance-scaled noise.
"""

import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class AddNoiseToGroundTruthOdom(Node):
    def __init__(self):
        super().__init__("add_noise_to_ground_truth_odom")
        self.get_logger().warn(
            "add_noise_to_ground_truth_odom.py is a stub — the original "
            "implementation was not checked in. This node will forward "
            "ground-truth odometry with a simple diagonal Gaussian "
            "position noise. Replace with the full noise model when "
            "porting the original ROS1 node."
        )
        self.declare_parameter("position_sigma", 0.05)
        self.declare_parameter("seed", 0)
        self._sigma = float(self.get_parameter("position_sigma").value)
        seed = int(self.get_parameter("seed").value)
        if seed != 0:
            random.seed(seed)
        self._pub = self.create_publisher(Odometry, "~/odom_noisy", 10)
        self._sub = self.create_subscription(
            Odometry, "~/odom_gt", self._on_odom, 10
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        out.pose.pose.position.x += random.gauss(0.0, self._sigma)
        out.pose.pose.position.y += random.gauss(0.0, self._sigma)
        out.pose.pose.position.z += random.gauss(0.0, self._sigma)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AddNoiseToGroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
