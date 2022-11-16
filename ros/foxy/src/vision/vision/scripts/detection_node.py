#!/usr/bin/env  /usr/bin/python3
import sys
import signal
import argparse
import time
import threading
import cv2

from tkinter.tix import Tree
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import detect
import tflite_runtime.interpreter as tflite
import platform
import subprocess
from PIL import ImageDraw
from PIL import Image as imag1
from scipy.signal import savgol_filter
import numpy as np
import imutils
import v4l2capture
import os
import select
import rclpy
from rclpy.node import Node
import simplejson
from vision_interfaces.msg import Boundingbox   # CHANGE
from std_msgs.msg import String

MODEL = '/home/fiborobotlab/ros/foxy/src/vision/vision/models/output_tflite_graph_edgetpu.tflite'
LABEL = '/home/fiborobotlab/ros/foxy/src/vision/vision/models/label.txt'
THRESHOLD = 0.4
COUNT = 1 # Number of times to run inference
width, height = 1280, 720
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('vision'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
Ball_position_topic='/{}/Ball_position'.format(robotname)
Robot_state_topic='/{}/Robot_state'.format(robotname)
Follow_ball_state_topic='/{}/Follow_ball_state'.format(robotname)
camera0_topic='/{}/camera0'.format(robotname)
detection_state_topic = '/{}/detection_state'.format(robotname)
test_topic = '/{}/test'.format(robotname)
stainding_tipic='/{}/standing_status'.format(robotname)
kick_topic='/{}/kick_status'.format(robotname)


SetPosition_topic='/{}/set_position'.format(robotname)
GetPosition_topic='/{}/get_position'.format(robotname)
Ball_position_kick_topic='/{}/Ball_position_kick'.format(robotname)
action_state_topic='/{}/action_state'.format(robotname)
head_state_topic='/{}/Head_position'.format(robotname)
goal_position_topic = '/{}/goal_position'.format(robotname)
get_angle_topic = '/{}/get_angle'.format(robotname)

controller_state_topic='{}/controller_state'.format(robotname)
action_state_topic='/{}/action_state'.format(robotname)

class Detection_Node(Node):
  def __init__(self):
    super().__init__('Detection_Node')
    self.init_variable()  
    self.subscriber()
    self.publisher()
    self.Boundingbox=Boundingbox()
    self.String=String()



  def init_variable(self):
    self.center_x_pub,self.center_y_pub=width/2, height/2 
    self.labels = self.load_labels(LABEL)
    self.interpreter = self.make_interpreter(MODEL)
    self.interpreter.allocate_tensors()
    self.bridge = CvBridge()
    self.time_ball_found=0
    self.time_ball_not_found=0
    self.prev_position=[]
  def publisher(self):
    self.publisher_Ball = self.create_publisher(Boundingbox, Ball_position_topic, 10)     # CHANGE
    self.publisher_Head_state = self.create_publisher(String, '/Head_state', 10)     # CHANGE

  def subscriber(self):	
    self.camera0_sub = self.create_subscription(Image, camera0_topic, self.detection, 5)  
    self.camera0_sub
  def load_labels(self,path, encoding='utf-8'): #Returns: Dictionary mapping indices to labels.
    with open(path, 'r', encoding=encoding) as f:
      lines = f.readlines()
      if not lines:
        return {}
      if lines[0].split(' ', maxsplit=1)[0].isdigit():
        pairs = [line.split(' ', maxsplit=1) for line in lines]
        return {int(index): label.strip() for index, label in pairs}
      else:
        return {index: line.strip() for index, line in enumerate(lines)}

  def make_interpreter(self,model_file):
    model_file, *device = model_file.split('@')
    return tflite.Interpreter(
        model_path=model_file,
        experimental_delegates=[
            tflite.load_delegate('libedgetpu.so.1', {'device': device[0]} if device else {})])

  def draw_objects(self,draw, objs, labels):
    """Draws the bounding box and label for each object."""
    for obj in objs:
      bbox = obj.bbox
      draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                    outline='blue')
      draw.text((bbox.xmin + 10, bbox.ymin + 10), '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), fill='blue')
  def object_detected(self,objs,image):
    for obj in objs:
        if obj.id == 0 and obj.score > 0.7:
           self.time_ball_found=time.time()
           center_x = (obj.bbox.xmax + obj.bbox.xmin)/2
           center_y = (obj.bbox.ymax + obj.bbox.ymin)/2	
           if self.prev_position!=[]:
              if self.prev_position[0]:
                 if abs(center_x - self.prev_position[0]) >5:
                    self.center_x_pub=center_x
                    self.prev_position[0]=center_x
                 else:
                    self.center_x_pub= self.prev_position[0]
              if self.prev_position[1]:
                 if abs(center_y - self.prev_position[1]) >5:
                    self.center_y_pub=center_y
                    self.prev_position[1]=center_y
                 else:
                    self.center_y_pub= self.prev_position[1]
           else:
              self.prev_position.append(center_x)
              self.prev_position.append(center_y)
           ##########################################################################
           self.String.data="Ball_tracking"
           self.publisher_Head_state.publish(self.String)
           self.get_logger().info('Publishing center x: %d center y: %d ' % (self.center_x_pub,self.center_y_pub))
           self.Boundingbox.ball.x=int(self.center_x_pub)
           self.Boundingbox.ball.y=int(self.center_y_pub)
           self.publisher_Ball.publish(self.Boundingbox)
        else:
          if time.time()-self.time_ball_found >2:
            print('1')
            self.String.data="Scanning"
            self.publisher_Head_state.publish(self.String)
    if time.time()-self.time_ball_found >2:
      print('1')
      self.String.data="Scanning"
      self.publisher_Head_state.publish(self.String)
    image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)
    cv2.imshow("S",image)
    cv2.waitKey(1)
  
  def detection(self,image):
    image=self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    image = imag1.fromarray(image)
    if image.size != (width, height):
        image = image.resize((width, height))
    scale = detect.set_input(self.interpreter, (width, height), lambda size: image.resize(size, imag1.ANTIALIAS))
    inference_time = 0
    for _ in range(COUNT):
        start = time.perf_counter()
        self.interpreter.invoke()
        inference_time += time.perf_counter() - start
        objs = detect.get_output(self.interpreter, THRESHOLD, scale)
        # print('%.2f ms' % (inference_time * 1000))
        self.draw_objects(ImageDraw.Draw(image), objs, self.labels)
    self.object_detected(objs,image)

  
def main(args=None):
    rclpy.init(args=args)
    Detection = Detection_Node()
    while rclpy.ok():
        rclpy.spin_once(Detection)
    Detection.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
