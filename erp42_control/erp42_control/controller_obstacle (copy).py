#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String
from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from stanley import Stanley
from erp42_msgs.msg import ControlMessage
from tf_transformations import *
import numpy as np
import math as m
import threading
from pyproj import *



class PathHandler():
    def __init__(self, node):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(PoseArray, "right_path/obstacle", self.callback_path, qos_profile)
        node.create_subscription(PoseArray, "left_path/obstacle", self.callback_path, qos_profile)
        node.create_subscription(PoseArray, "path/global", self.callback_path, qos_profile)
        node.create_subscription(PoseArray, "path/smooth", self.callback_path, qos_profile)

        self.cx = []
        self.cy = []
        self.cyaw = []

    def callback_path(self,msg):
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.position.x)
            cy.append(p.position.y)
            _,_,yaw = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]) 
            cyaw.append(yaw)    
        return cx, cy, cyaw
    
class State():
    def __init__(self, node, odom_topic):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Odometry, odom_topic, self.callback, qos_profile)

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    
    
        
class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 0, 20))
    
class SpeedSupporter():
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res
    
class Drive():
    def __init__(self, node, state, path):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Float32, "target_right_speed", self.callback_speed, qos_profile)
        node.create_subscription(Float32, "target_left_speed", self.callback_speed, qos_profile)
        node.create_subscription(Float32, "target_global_speed", self.callback_speed, qos_profile)
        node.create_subscription(Float32, "target_smooth_speed", self.callback_speed, qos_profile)
        node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile)

        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile)
        self.pub_idx = node.create_publisher(Int32, "target_idx", qos_profile)

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.path = path
        self.state = state
        self.last_target_idx = 0
        self.path_speed = 6.0
        self.max_steer = 28
        self.current_speed = 0.


    def publish_cmd(self):

        print(self.state.v, self.current_speed)

        steer, target_idx, hdr, ctr = self.st.stanley_control(self.state, self.path.cx, self.path.cy, self.path.cyaw, self.last_target_idx)
        self.last_target_idx = target_idx
        adapted_speed = self.ss.adaptSpeed(self.path_speed, hdr, ctr, min_value=4, max_value=20)
        steer = np.clip(steer,m.radians((-1)*self.max_steer),m.radians(self.max_steer))
        speed = self.pid.PIDControl(self.state.v * 3.6, adapted_speed)
        if self.state.v * 3.6 >= adapted_speed:
            input_brake = (abs(self.state.v - adapted_speed) / 20.0) * 200
        else:
            input_brake = 0

        msg = ControlMessage()
        msg.speed = speed*10
        msg.steer = int(m.degrees((-1)*steer))
        msg.gear = 2
        msg.brake = int(input_brake)

        data = Int32()
        data.data = target_idx

        self.pub.publish(msg)
        self.pub_idx.publish(data)

    
    def callback_speed(self,msg):
        self.path_speed = msg.data

    def callback_erp(self, msg):
        self.current_speed = msg.speed



def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("driving_node")
    state = State(node, "/localization/kinematic_state")
    path_tracking = PathHandler(node)
    d = Drive(node, state, path_tracking)


    thread = threading.Thread(target=rclpy.spin, args= (node, ), daemon = True)
    thread.start()

    rate = node.create_rate(8)

    while rclpy.ok():
        try:
            d.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()
    
if __name__=="__main__":
    main()