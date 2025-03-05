#!/usr/bin/env python
#Created by Tanishq Dwivedi
import time
import subprocess
import threading
from pathlib import Path
import datetime
from datetime import datetime
import rclpy
from rclpy.node import Node
import serial
import os
from std_msgs.msg import String
import io
import string
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.5)
#Class to start the Publisher node -> 'research_node' The queue is 10
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('manipulation_node')
        self.rpublisher_ = self.create_publisher(String, 'manipulationData', 10)
        
        
    #publishing the actual line of the sensor data from Arduino with ROS
    def publish_line(self, line):
        msg = String()
        msg.data = line
        # need to parse it into a variable -> depth
        self.rpublisher_.publish(msg)
        self.depthpublisher.publish(msg)
        print(line)

#start the c++ files before we start the ROS node and publish the data
def main():
        ser.reset_input_buffer()
        line = ""
        while line != "Servo initialized at 90 degrees":
            line = ser.readline().decode('utf-8').rstrip()
        rclpy.init()
        minimal_publisher = MinimalPublisher()
        rclpy.spin(getData(minimal_publisher))


#The ROS Node will keep running and this function will read off the 
#Arduino data
def getData(minimal_publisher):
    push_command = input()
    while True:
            ser.write(push_command)
            line = ser.readline().decode('utf-8').rstrip()
            minimal_publisher.publish_line(line)
            push_command = input()
            



t2 = threading.Thread(target = main)
t2.start()