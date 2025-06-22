#!/usr/bin/env python3  
import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import PoseWithCovarianceStamped  
import time  
  
class InitialPoseSetter(Node):  
    def __init__(self):  
        super().__init__('initial_pose_setter')  
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)  
        # 延迟发布以确保AMCL已启动  
        self.timer = self.create_timer(5.0, self.publish_initial_pose)  
          
    def publish_initial_pose(self):  
        msg = PoseWithCovarianceStamped()  
        msg.header.frame_id = "map"  
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.pose.pose.position.x = -2.0  
        msg.pose.pose.position.y = -2.0  
        msg.pose.pose.position.z = 0.0  
        msg.pose.pose.orientation.w = 1.0  
          
        self.publisher.publish(msg)  
        self.get_logger().info('Initial pose published')  
        self.timer.cancel()  # 只发布一次  
  
def main():  
    rclpy.init()  
    node = InitialPoseSetter()  
    rclpy.spin(node)  
    rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()