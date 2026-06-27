#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        
        arm_controller.joint_names = ["base_link_link1_to_base_link", "link1_link2_to_link1", "link2_link3_to_link2", "link3_link4_to_link3", "link4_link5_to_link4", "link5_link6_to_link5"]
        gripper_controller.joint_names = ["AX_12A_04_Revolute-34"]
        
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        
        arm_goal.positions = msg.position[:6]
        gripper_goal.positions = [msg.position[6]]
        
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main():
    rclpy.init()
    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()