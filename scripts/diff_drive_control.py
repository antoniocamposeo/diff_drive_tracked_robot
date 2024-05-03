#!/usr/bin/env python3
"""
Motor Control Differential Drive Node 
Convert velocity command from teleop with keyboard or joy into command 
for sabertooth driver.

ROS 
Subscriber --> /cmd_vel 
Publisher  --> None 
"""

import serial.tools.list_ports as port 
import time
import rospy

from geometry_msgs.msg import Twist 
from pysabertooth import Sabertooth


class Motor: 
    def __init__(self, name_driver, baudrate, address, timeout):
        # Driver
        self.name_driver =  name_driver
        self.BAUDRATE = baudrate
        self.ADDRESS = address
        self.TIMEOUT = timeout
        self.Driver = None # obj get from pysabertooth

        # Robot constants (physical properties of the robot)
        self.WHEEL_RADIUS = 1  # radius of wheels (meters)
        self.WHEEL_SEPARATION = 1  # width of the robot (meters)

        # Data from cmd_vel 
        self.linear_speed = None # linear velocity - command forward or backward 
        self.angular_speed = None
        self.max_linear_speed = 0.5 # m/s
        self.max_angular_speed = 1.5 # rad/s
        
        # Data from Driver 
        self.vel_r = None # Right motor power  
        self.vel_l = None # Left motor power
        self.max_vel_l =  (self.max_linear_speed) - (self.max_angular_speed * self.WHEEL_SEPARATION) / (2.0*self.WHEEL_RADIUS)
        self.max_vel_r =  (self.max_linear_speed) + (self.max_angular_speed * self.WHEEL_SEPARATION) / (2.0*self.WHEEL_RADIUS)

        # ROS  
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.callback_ros, queue_size=1)

    def init_driver(self):
        rospy.loginfo("Detecting Sabertooth Driver")    
        portlist = list(port.comports())
        serial_port = ""
        try:
            for p in portlist:
                if self.name_driver in str(p):
                    serial_port = str(p).split(" ")
        except Exception as e:
            rospy.logerr(e)
        finally:
            self.Driver = Sabertooth(serial_port[0], self.BAUDRATE, self.ADDRESS, self.TIMEOUT)

        rospy.loginfo("Connected to Driver!!")

    def callback_ros(self,msg):
        #self.linear_speed = msg.linear.x
        #self.angular_speed = msg.angular.z
        self.linear_speed = 0.4
        self.angular_speed = 0.5
        
        # convert linear and angular inputs to left and right wheel velocities
        self.vel_l = (self.linear_speed) - (self.angular_speed ) / (2.0)

        self.vel_r = (self.linear_speed) + (self.angular_speed ) / (2.0)
        
        rospy.loginfo("cmd command: linear: %f, Angular:%f", self.linear_speed, self.angular_speed)
        rospy.loginfo("motor command not mapped: R: %f, L:%f", self.vel_l, self.vel_r)
        # map values obtained above  [-in_min , in_max] out of [-out_min, out_max]
        self.vel_l = self.map_val(self.vel_l, -self.max_vel_r, self.max_vel_r, -10, 10)
        self.vel_r = self.map_val(self.vel_r, -self.max_vel_r, self.max_vel_r, -10, 10)

        rospy.loginfo("mapped commands: R: %f, L:%f", self.vel_l, self.vel_r)
        print("---------------------")
        # send command to driver    
        #self.Driver.driveBoth(self.vel_l, self.vel_r) 

        
    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("sabertooth_diffdrive_control")
    control_obj = Motor(name_driver="Sabertooth", baudrate=9600, address=128, timeout=0.1)
    #control_obj.init_driver()
    time.sleep(2)
    control_obj.main()