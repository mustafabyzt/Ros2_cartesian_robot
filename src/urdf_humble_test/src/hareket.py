#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('joint_trajectory_hareket')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

       
        self.hareket_1 = 1.0
        self.hareket_2 = 1.0
        self.hareket_3 = 1.0

        
        self.get_user_inputs()

    def get_user_inputs(self):
        self.hareket_1 = float(input("X ekseninde ne kadar hareket etsin: "))
        self.hareket_2 = float(input("Y ekseninde ne kadar hareket etsin: "))
        self.hareket_3 = float(input("Z ekseninde ne kadar hareket etsin: "))
        

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = ['x_axis_joint', 'y_axis_joint', 'welding_head_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.hareket_1, self.hareket_2, self.hareket_3]
        point.time_from_start.sec = 10
        point.time_from_start.nanosec = 0
        msg.points = [point]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointTrajectory message: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

