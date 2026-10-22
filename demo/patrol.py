"""Keeps the demo robot moving: sends Nav2 goals around the map's ring of open
space, one after another, with a pause at each, so Goals, the behavior tree,
the map and the logs always have something to show. Exits when Nav2 fails
five goals in a row, so the machine starts over."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# (x, y) in the map frame of nav2_bringup's tb3_sandbox map, each at least
# 0.65 m from the walls and pillars.
WAYPOINTS = [(0.6, -1.85), (1.8, 0.35), (-0.6, 1.75), (-2.0, -0.05)]
PAUSE_S = 20
GOAL_TIMEOUT_S = 180
SUCCEEDED = 4
MAX_FAILURES = 5


def goal_at(index):
    x, y = WAYPOINTS[index % len(WAYPOINTS)]
    next_x, next_y = WAYPOINTS[(index + 1) % len(WAYPOINTS)]
    yaw = math.atan2(next_y - y, next_x - x)  # facing the next waypoint
    goal = NavigateToPose.Goal()
    # The stamp stays 0, meaning the latest transform: the simulation's clock
    # isn't this node's.
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return goal, (x, y)


def main():
    rclpy.init()
    node = Node('demo_patrol')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=180):
        print('[patrol] navigate_to_pose never came up', flush=True)
        return 1
    failures = 0
    index = 0
    while rclpy.ok():
        goal, point = goal_at(index)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=30)
        handle = sent.result() if sent.done() else None
        reached = False
        if handle is not None and handle.accepted:
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result, timeout_sec=GOAL_TIMEOUT_S)
            if result.done():
                reached = result.result().status == SUCCEEDED
            else:
                handle.cancel_goal_async()
        failures = 0 if reached else failures + 1
        print(f'[patrol] {point}: {"reached" if reached else "not reached"}', flush=True)
        if failures >= MAX_FAILURES:
            print(f'[patrol] {MAX_FAILURES} goals in a row not reached', flush=True)
            return 1
        index += 1
        until = time.monotonic() + PAUSE_S
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.5)
    return 0


if __name__ == '__main__':
    sys.exit(main())
