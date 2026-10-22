"""Gives AMCL the robot's spawn pose (-2.0, -0.5, facing +x), which the
simulation's launch file puts it at, until AMCL reports a pose. Exits 1 if it
never does."""
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

rclpy.init()
node = Node('demo_initial_pose')
publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
reported = []
node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', reported.append,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
for _ in range(120):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x, pose.pose.pose.position.y = -2.0, -0.5
    pose.pose.pose.orientation.w = 1.0
    pose.pose.covariance[0] = pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.07
    publisher.publish(pose)
    rclpy.spin_once(node, timeout_sec=1.0)
    if reported:
        print('[demo] AMCL has the robot\'s pose', flush=True)
        sys.exit(0)
sys.exit(1)
