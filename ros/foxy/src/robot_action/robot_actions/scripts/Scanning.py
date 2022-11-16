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
from robot_actions_interfaces.msg import DynamixelRequest,Position,Feedback
from std_msgs.msg import String
from vision_interfaces.msg import Boundingbox 
from pid_control import PID_Control
joint_limit_pan=[-1.0, 1.0]
joint_limit_tilt=[0.0, 1.0]
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('vision'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
path_1,path_2,path_3,path_4,path_5,path_6,path_7=[0.8,0.9],[-0.8,0.9],[0.8,0.5],[-0.8,0.5],[0.8,0.1],[-0.8,0.1],[0.8,0.9]
Ball_position_topic='/{}/Ball_position'.format(robotname)
kp_pan = 0.05
ki_pan = 0
kd_pan = 0.0001

kp_tilt = 0.06
ki_tilt = 0
kd_tilt = 0.0001

dt = 0.01
screen_size = [1280, 720]
class Scanning(Node):
    def __init__(self):
        super().__init__('Scanning_Node')
        timer_period = 1.5  # seconds
        timer_period2= 0.6 # seconds
        self.timer = self.create_timer(timer_period, self.timer_request_scan)
        self.timer2 = self.create_timer(timer_period2, self.timer_reqest_balltracking)
        self.publisher = self.create_publisher(DynamixelRequest,'/joint_request',10) 
        self.subscription = self.create_subscription(String,'/Head_state',self.listener_callback,10)        
        self.subscription_ball = self.create_subscription(Boundingbox,Ball_position_topic,self.ball_position,10)        
        self.subscription_feedback= self.create_subscription(Feedback,'/feedback',self.feedback,10)        

        self.State="Scanning"
        self.i=0 
        self.object_position_x=0.0
        self.object_position_y=0.0
        self.screen_size = screen_size
        self.pid_pan = PID_Control(kp_pan, ki_pan, kd_pan,dt)
        self.pid_tilt = PID_Control(kp_tilt, ki_tilt, kd_tilt, dt)
        self.screen_center_x = self.screen_size[0]/2
        self.screen_center_y = self.screen_size[1]/2
        self.current_position=[0.0,0.0]
        self.motor_position_next_x=0.0
    def feedback(self,msg):
        self.current_position=msg.current_position
        # print(self.current_position)
    
    def ball_position(self,msg):
        self.object_position_x=msg.ball.x
        self.object_position_y=msg.ball.y
        self.motor_position_next_x=self.current_position[0]+(int(10*(3.14/180)*(self.pid_pan.update(self.screen_center_x, self.object_position_x)))/10)
        self.motor_position_next_y=self.current_position[1]+(-1*(int(10*(3.14/180)*(self.pid_tilt.update(self.screen_center_y, self.object_position_y)))/10))
        
        if self.motor_position_next_x < joint_limit_pan[0]:
           self.motor_position_next_x = joint_limit_pan[0]
        if self.motor_position_next_x > joint_limit_pan[1]:
           self.motor_position_next_x = joint_limit_pan[1]
        if self.motor_position_next_y < joint_limit_tilt[0]:
           self.motor_position_next_y = joint_limit_tilt[0]
        if self.motor_position_next_y > joint_limit_tilt[1]:
           self.motor_position_next_y = joint_limit_tilt[1]
        
        print(self.motor_position_next_x)
        print(self.motor_position_next_y)

    def listener_callback(self, msg):
        self.State=msg.data
        print(self.State)
    
    def timer_reqest_balltracking(self):
        if self.State=="Ball_tracking":
            msg = DynamixelRequest()
            msg_Position=Position()
            msg_Position.position=[self.motor_position_next_x,self.motor_position_next_y]
            msg.joints=["joint1","joint2"]
            msg.sec=0.5
            msg.positions.append(msg_Position)
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)

    def timer_request_scan(self):
        if self.State == "Scanning":
            self.i+=1
            msg = DynamixelRequest()
            msg_Position=Position()
            msg.joints=["joint1","joint2"]
            msg.sec=1.0
            if self.i==1:
                msg_Position.position=path_1
            elif self.i==2:
                msg_Position.position=path_2
            elif self.i==3:
                msg_Position.position=path_3
            elif self.i==4:
                msg_Position.position=path_4
            elif self.i==5:
                msg_Position.position=path_5
            elif self.i==6:
                msg_Position.position=path_6
            elif self.i==7:
                msg_Position.position=path_7
                self.i=1
            msg.positions.append(msg_Position)
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        

def main(args=None):
    rclpy.init(args=args)
    Scanning_client = Scanning()
    rclpy.spin(Scanning_client)
if __name__ == '__main__':
    main()