#!/usr/bin/env  /usr/bin/python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import subprocess
import simplejson
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from robot_actions_interfaces.msg import DynamixelRequest,Feedback
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('vision'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
Goal_succeeded='Not_Done!!'
task_count=0
task_status="Empty"

class DynamixelInterfaces(Node):
    def __init__(self):
        super().__init__('DynamixelInterfaces_Node')
        print(DynamixelRequest())
        self.task_status="Empty"
        self.subscription = self.create_subscription(DynamixelRequest,'/joint_request',self.request_callback,10)        
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.publisher = self.create_publisher(Feedback,'/feedback',10) 
        self.current=[0.0,0.0]
        self._action_client.wait_for_server()
    
    def request_callback(self,msg):
        self.send_goal(msg.joints,msg.positions[0].position,msg.sec)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._goal_handle=goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)        
    
    def feedback_callback(self, feedback):
        Feedback_msg=Feedback()
        self.get_logger().info('Received feedback_position : {}'.format(feedback.feedback.desired.positions))
        self.current[0]=(int(feedback.feedback.desired.positions[0]*100)/100)
        self.current[1]=(int(feedback.feedback.desired.positions[1]*100)/100)
        Feedback_msg.current_position=self.current
        print(self.current)
        self.publisher.publish(Feedback_msg)
        # self.get_logger().info('Received feedback_velocities : {}'.format(feedback.feedback.desired.velocities))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        # Shutdown after receiving a result
    def send_goal(self,joints,path,sec=3):
        goal_traject=FollowJointTrajectory.Goal()
        goal_traject.trajectory.joint_names=joints
        joint_point=JointTrajectoryPoint()
        joint_point.positions=path
        goal_traject.trajectory.points.append(joint_point)

        if sec < 1 :
            joint_point.time_from_start.nanosec= int(sec*1000000000)
            print(int(sec*1000000000))
            joint_point.time_from_start.sec= 0
        else:
            joint_point.time_from_start.sec= int(sec)
            joint_point.time_from_start.nanosec= 0
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_traject,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
def main(args=None):
    rclpy.init(args=args)
    action_client = DynamixelInterfaces()
    action_client.send_goal(joints=["joint1","joint2"],path=[0.0,0.0])   
    rclpy.spin(action_client)
if __name__ == '__main__':
    main()
#-1 1 : [0]
# 0.0 1.0 :[1]