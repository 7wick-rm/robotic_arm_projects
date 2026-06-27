#!/usr/bin/env python3
"""
Action client CLI for the pick_and_place action server.

Usage:
  ros2 run your_package pick_and_place_client.py R 0 0.3 -1.6 0.02
  ros2 run your_package pick_and_place_client.py <color> <block_id> <place_x> <place_y> <place_z>

Arguments:
  color     : "R", "G", or "B"
  block_id  : integer block id from perception
  place_x/y/z : place target position in base_link frame (metres)

Unlike a service client, this prints live feedback while the robot is moving.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mtc_tutorial.action import PickAndPlace   # replace your_package


class PickAndPlaceClient(Node):
    def __init__(self):
        super().__init__("pick_and_place_client")
        self._client = ActionClient(self, PickAndPlace, "pick_and_place")

    def send_goal(self, color: str, block_id: int,
                  place_x: float, place_y: float, place_z: float):

        self.get_logger().info("Waiting for action server ...")
        self._client.wait_for_server()

        goal = PickAndPlace.Goal()
        goal.color    = color
        goal.block_id = block_id
        goal.place_x  = place_x
        goal.place_y  = place_y
        goal.place_z  = place_z

        self.get_logger().info(
            f"Sending goal → color: {color} | block_id: {block_id} | "
            f"place: [{place_x}, {place_y}, {place_z}]"
        )

        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by server (block color not detected yet?)")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal ACCEPTED — waiting for result ...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        # Prints live updates while the robot is planning / executing
        self.get_logger().info(f"[Feedback] {feedback_msg.feedback.status}")

    def _result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"✓ Success: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed:  {result.message}")
        rclpy.shutdown()


def main():
    if len(sys.argv) != 6:
        print(__doc__)
        sys.exit(1)

    color    = sys.argv[1]
    block_id = int(sys.argv[2])
    place_x  = float(sys.argv[3])
    place_y  = float(sys.argv[4])
    place_z  = float(sys.argv[5])

    if color not in ("R", "G", "B"):
        print(f"Error: color must be R, G, or B — got '{color}'")
        sys.exit(1)

    rclpy.init()
    client = PickAndPlaceClient()
    client.send_goal(color, block_id, place_x, place_y, place_z)
    rclpy.spin(client)


if __name__ == "__main__":
    main()
