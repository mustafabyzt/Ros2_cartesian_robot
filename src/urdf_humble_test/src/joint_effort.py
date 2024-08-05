#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort

def main(args=None):
    rclpy.init(args=args)
    node = Node('apply_effort_client')
    
    effort_value = float(input("Uygulanacak kuvvet değerini girin: "))
    client = node.create_client(ApplyJointEffort, '/apply_joint_effort')
    
   
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service not available, shutting down...')
        rclpy.shutdown()
        return
    
    
    request_x = ApplyJointEffort.Request()
    request_x.joint_name = 'x_axis_joint'  
    request_x.effort = effort_value        
    request_x.start_time.sec = 15    
    request_x.start_time.nanosec = 0     
    request_x.duration.sec = 10           
    request_x.duration.nanosec = 0 
    
    request_y = ApplyJointEffort.Request()
    request_y.joint_name = 'y_axis_joint'  
    request_y.effort = effort_value        
    request_y.start_time.sec = 30 
    request_y.start_time.nanosec = 0     
    request_y.duration.sec = 10           
    request_y.duration.nanosec = 0  
    
    
    future_x = client.call_async(request_x)
    
    future_y = client.call_async(request_y)
    
    rclpy.spin_until_future_complete(node, future_x)
    rclpy.spin_until_future_complete(node, future_y)
   
    if future_x.result() is not None and future_y.result() is not None:
        node.get_logger().info('Response x_axis_joint: %s' % future_x.result().success)
        node.get_logger().info('Response y_joint: %s' % future_y.result().success)
    else:
        node.get_logger().error('Service call failed')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
