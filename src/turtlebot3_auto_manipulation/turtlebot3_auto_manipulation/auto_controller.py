#!/usr/bin/env python3      
      
import rclpy      
from rclpy.node import Node      
from rclpy.action import ActionClient       
from nav2_msgs.action import NavigateToPose      
from moveit_msgs.action import MoveGroup    
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions    
from control_msgs.action import GripperCommand      
import threading    
import time    
      
class AutoController(Node):      
    def __init__(self):      
        super().__init__('auto_controller')      
              
        # 参数      
        self.declare_parameters(      
            namespace='',      
            parameters=[      
                ('target_x1', 1.5),      
                ('target_y1', 1.0),      
                ('target_w1', 1.0),    
                ('target_x2', 1.5),      
                ('target_y2', 0.5),      
                ('target_w2', 1.0),   
            ]      
        )      
              
        # Action客户端      
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')      
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')      
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')      
              
        # 等待服务      
        self.get_logger().info('等待导航和机械臂服务...')      
        self.nav_client.wait_for_server()      
        self.move_group_client.wait_for_server()      
        self.gripper_client.wait_for_server()     
              
        # 开始任务序列      
        self.timer = self.create_timer(2.0, self.start_sequence)      
        self.sequence_started = False      
          
    def start_sequence(self):      
        if not self.sequence_started:      
            self.sequence_started = True      
            self.timer.cancel()     
            # 在新线程中运行任务序列    
            thread = threading.Thread(target=self.execute_task_sequence)    
            thread.daemon = True    
            thread.start()    
          
    def execute_task_sequence(self):      
        """执行完整的任务序列"""      
        try:      
            # 初始化位置    
            self.get_logger().info('导航回到初始位置...')      
            self.navigate_to_pose_sync(-2.0, -0.5, 1.0)  
            self.get_logger().info('机械臂回到初始位置...')      
            if self.move_arm_to_home_position():  
                self.get_logger().info('机械臂回到home位置成功')  
            else:  
                self.get_logger().error('机械臂回到home位置失败')  
            # 1. 导航到目标位置1      
            self.get_logger().info('开始导航到目标位置1...')      
            self.navigate_to_pose_sync(      
                self.get_parameter('target_x1').value,      
                self.get_parameter('target_y1').value,    
                self.get_parameter('target_w1').value      
            )      
                  
            # 2. 机械臂移动到抓取位置      
            self.get_logger().info('机械臂移动到抓取位置...')      
            if self.move_arm_to_pickup_position():  
                self.get_logger().info('机械臂移动到抓取位置成功')  
            else:  
                self.get_logger().error('机械臂移动到抓取位置失败')  
                return  
                  
            # 3. 抓取      
            self.get_logger().info('执行抓取...')      
            self.close_gripper_sync()      

            # 4. 导航到目标位置2      
            self.get_logger().info('开始导航到目标位置2...')      
            self.navigate_to_pose_sync(      
                self.get_parameter('target_x2').value,      
                self.get_parameter('target_y2').value,    
                self.get_parameter('target_w2').value      
            ) 

            # 5. 移动到释放位置      
            self.get_logger().info('移动到释放位置...')      
            if self.move_arm_to_release_position():  
                self.get_logger().info('机械臂移动到释放位置成功')  
            else:  
                self.get_logger().error('机械臂移动到释放位置失败')  
                  
            # 6. 释放      
            self.get_logger().info('释放物体...')      
            self.open_gripper_sync()      
                  
            # 7. 机械臂回到初始位置      
            self.get_logger().info('机械臂回到初始位置...')      
            if self.move_arm_to_home_position():  
                self.get_logger().info('机械臂回到home位置成功')  
            else:  
                self.get_logger().error('机械臂回到home位置失败')  
                  
            # 8. 导航回到初始位置      
            self.get_logger().info('导航回到初始位置...')      
            self.navigate_to_pose_sync(-2.0, -0.5, 1.0)      
                  
            self.get_logger().info('任务序列完成！')      
                  
        except Exception as e:      
            self.get_logger().error(f'任务执行失败: {str(e)}')      
          
    def navigate_to_pose_sync(self, x, y, w):      
        """同步导航到指定位置"""      
        goal_msg = NavigateToPose.Goal()      
        goal_msg.pose.header.frame_id = 'map'      
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()      
        goal_msg.pose.pose.position.x = x      
        goal_msg.pose.pose.position.y = y      
        goal_msg.pose.pose.position.z = 0.0    
        goal_msg.pose.pose.orientation.x = 0.0    
        goal_msg.pose.pose.orientation.y = 0.0    
        goal_msg.pose.pose.orientation.z = 0.0    
        goal_msg.pose.pose.orientation.w = w      
              
        future = self.nav_client.send_goal_async(goal_msg)      
            
        # 使用轮询等待而不是spin_until_future_complete    
        while not future.done():    
            time.sleep(0.1)    
            
        if future.result() and future.result().accepted:      
            result_future = future.result().get_result_async()      
            # 同样使用轮询等待结果    
            while not result_future.done():    
                time.sleep(0.1)    
            self.get_logger().info('导航完成')    
        else:    
            self.get_logger().error('导航目标被拒绝')    
  
    def move_arm_to_home_position(self):  
        """移动到预定义的home位置"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "arm"  
        goal_msg.request.num_planning_attempts = 10  
        goal_msg.request.allowed_planning_time = 5.0  
        goal_msg.request.max_velocity_scaling_factor = 0.1  
        goal_msg.request.max_acceleration_scaling_factor = 0.1  
          
        # 使用预定义的home状态关节值  
        constraint = Constraints()  
        joint_positions = [  
            ("joint1", 0.0),  
            ("joint2", -1.0),   
            ("joint3", 0.7),  
            ("joint4", 0.3)  
        ]  
          
        for joint_name, position in joint_positions:  
            jc = JointConstraint()  
            jc.joint_name = joint_name  
            jc.position = position  
            jc.tolerance_above = 0.01  
            jc.tolerance_below = 0.01  
            jc.weight = 1.0  
            constraint.joint_constraints.append(jc)  
          
        goal_msg.request.goal_constraints = [constraint]  
        goal_msg.planning_options.plan_only = False  
          
        return self._execute_move_group_goal(goal_msg)  
  
    def move_arm_to_pickup_position(self):  
        """移动到抓取准备位置"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "arm"  
        goal_msg.request.num_planning_attempts = 10  
        goal_msg.request.allowed_planning_time = 5.0  
        goal_msg.request.max_velocity_scaling_factor = 0.1  
        goal_msg.request.max_acceleration_scaling_factor = 0.1  
          
        # 设置一个在关节限制范围内的安全抓取位置  
        constraint = Constraints()  
        joint_positions = [  
            ("joint1", 0.5),    # 轻微旋转  
            ("joint2", -0.8),   # 肩部下降  
            ("joint3", 1.0),    # 肘部弯曲  
            ("joint4", 0.5)     # 腕部调整  
        ]  
          
        for joint_name, position in joint_positions:  
            jc = JointConstraint()  
            jc.joint_name = joint_name  
            jc.position = position  
            jc.tolerance_above = 0.01  
            jc.tolerance_below = 0.01  
            jc.weight = 1.0  
            constraint.joint_constraints.append(jc)  
          
        goal_msg.request.goal_constraints = [constraint]  
        goal_msg.planning_options.plan_only = False  
          
        return self._execute_move_group_goal(goal_msg)  
  
    def move_arm_to_release_position(self):  
        """移动到释放位置"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "arm"  
        goal_msg.request.num_planning_attempts = 10  
        goal_msg.request.allowed_planning_time = 5.0  
        goal_msg.request.max_velocity_scaling_factor = 0.1  
        goal_msg.request.max_acceleration_scaling_factor = 0.1  
          
        # 设置释放位置的关节值  
        constraint = Constraints()  
        joint_positions = [  
            ("joint1", -0.5),   # 反向旋转  
            ("joint2", -0.6),   # 肩部位置  
            ("joint3", 0.8),    # 肘部位置  
            ("joint4", 0.4)     # 腕部位置  
        ]  
          
        for joint_name, position in joint_positions:  
            jc = JointConstraint()  
            jc.joint_name = joint_name  
            jc.position = position  
            jc.tolerance_above = 0.01  
            jc.tolerance_below = 0.01  
            jc.weight = 1.0  
            constraint.joint_constraints.append(jc)  
          
        goal_msg.request.goal_constraints = [constraint]  
        goal_msg.planning_options.plan_only = False  
          
        return self._execute_move_group_goal(goal_msg)  
  
    def _execute_move_group_goal(self, goal_msg):  
        """执行MoveGroup目标的通用方法"""  
        future = self.move_group_client.send_goal_async(goal_msg)  
          
        while not future.done():  
            time.sleep(0.1)  
          
        if future.result() and future.result().accepted:  
            result_future = future.result().get_result_async()  
            while not result_future.done():  
                time.sleep(0.1)  
            return True  
        else:  
            return False  
          
    def close_gripper_sync(self):      
        """同步关闭夹爪"""      
        goal_msg = GripperCommand.Goal()      
        goal_msg.command.position = 0.0  # 关闭位置      
        goal_msg.command.max_effort = 10.0      
              
        future = self.gripper_client.send_goal_async(goal_msg)      
            
        # 使用轮询等待    
        while not future.done():    
            time.sleep(0.1)    
            
        if future.result() and future.result().accepted:      
            result_future = future.result().get_result_async()      
            while not result_future.done():    
                time.sleep(0.1)    
            self.get_logger().info('夹爪关闭完成')    
        else:    
            self.get_logger().error('夹爪关闭被拒绝')    
          
    def open_gripper_sync(self):      
        """同步打开夹爪"""      
        goal_msg = GripperCommand.Goal()      
        goal_msg.command.position = 0.02  # 打开位置      
        goal_msg.command.max_effort = 10.0      
              
        future = self.gripper_client.send_goal_async(goal_msg)      
            
        # 使用轮询等待    
        while not future.done():    
            time.sleep(0.1)    
            
        if future.result() and future.result().accepted:      
            result_future = future.result().get_result_async()      
            while not result_future.done():    
                time.sleep(0.1)    
            self.get_logger().info('夹爪打开完成')    
        else:    
            self.get_logger().error('夹爪打开被拒绝')    
      
def main(args=None):      
    rclpy.init(args=args)      
    node = AutoController()      
        
    try:    
        rclpy.spin(node)    
    except KeyboardInterrupt:    
        pass    
    finally:    
        node.destroy_node()    
        rclpy.shutdown()    
      
if __name__ == '__main__':      
    main()