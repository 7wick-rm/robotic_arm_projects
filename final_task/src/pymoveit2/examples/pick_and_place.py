#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters

        self.declare_parameter("target_color", "Q")
        self.target_color = self.get_parameter("target_color").value.upper()
        self.declare_parameter("x", 0.6)
        self.x = float(
            self.get_parameter("x").value
        )
        self.declare_parameter("y", 0.0)
        self.y = float(
            self.get_parameter("y").value
        )
        self.declare_parameter("z", 1.1)
        self.z = float(
            self.get_parameter("z").value
        )
        self.declare_parameter("approach_offset", 0.31)
        self.approach_offset = float(
            self.get_parameter("approach_offset").value
        )
        # Flags
        self.target_coords = None  # Stores the locked coordinates

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),  
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()
    
        self.coords_callback()

    def coords_callback(self):
        # Lock coordinates
        if (self.target_color == "R"):
            self.x=0.6
            self.y=0.0
            self.z=1.1
        if (self.target_color == "G"):
            self.x=0.6
            self.y=-0.206
            self.z=1.1
        if (self.target_color == "B"):
            self.x=0.6
            self.y=0.206
            self.z=1.1
        self.target_coords = [self.x, self.y, self.z]
        self.get_logger().info(
            f"Target locked at: "
            f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
        )

        # Use locked coordinates
        pick_position = [self.target_coords[0], self.target_coords[1], self.target_coords[2] - 0.60]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        # --- Pick-and-place sequence ---

        # 1. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above target (Cartesian)
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Move down to approach object
        approach_position = [
            pick_position[0],
            pick_position[1],
            pick_position[2] - self.approach_offset
        ]

        self.moveit2.move_to_pose(
            position=approach_position,
            quat_xyzw=quat_xyzw,
            cartesian=True
        )
        self.moveit2.wait_until_executed()

        # 5. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 7. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 8. Move to drop joint configuration
        self.moveit2.move_to_configuration(self.drop_joints)
        self.moveit2.wait_until_executed()

        # 9. Open gripper to release
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 10. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 11. Return to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Pick-and-place sequence complete.")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()