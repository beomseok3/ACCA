#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from erp42_msgs.msg import SerialFeedBack, ControlMessage

from stanley import Stanley
from DB import DB
import numpy as np
import math as m


from enum import Enum
import threading


# 범석, 진주 코드 나중에 import하기



def euler_from_quaternion(quaternion):
    
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)state_machine

    return qx, qy, qz, qw




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




# enum class의 각 state는 주행 순서대로 설정
class State(Enum):    
    A1A2 = "driving"
    A2A3 = "pickup"
    A3A4 = "driving"
    A4A5 = "obstacle"

    A5A6 = "driving"
    A6A7 = "traffic_light"
    A7A8 = "driving"
    A8A9 = "traffic_light"

    A9A10 = "driving"
    A10A11 = "obstacle"
    A11A12 = "driving"
    A12A13 = "traffic_light"

    A13A14 = "driving"
    A14A15 = "delivery"
    A15A16 = "driving"
    A16A17 = "traffic_light"

    A17A18 = "driving"
    A18A19 = "traffic_light"
    A19A20 = "driving"
    A20A21 = "traffic_light"

    A21A22 = "driving"
    A22A23 = "traffic_light"
    A23A24 = "driving"
    A24A25 = "parking"
    A25A26 = "driving"




class GetPath():
    def __init__(self, db, init_state):
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cv = []
        self.db = db

        self.file_open_with_id(init_state.name)


    def file_open_with_id(self, id):
        self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(id)



class GetOdometry():
    def __init__(self, node, odom_topic):
        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

        self.node = node

        self.node.create_subscription(Odometry, odom_topic, self.callback, qos_profile=qos_profile_system_default)
        self.node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile=qos_profile_system_default)

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    def callback_erp(self,msg):
        self.v = msg.speed

class StateMachine():
    def __init__(self, node, odometry, path, state):
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile=qos_profile_system_default)

        self.node = node
        self.state = state
        self.path = path
        self.odometry = odometry

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.target_speed = 0.
        self.adapted_speed = 0.
        self.reverse = False

        # ControlMessage
        self.speed = 0
        self.steer = 0
        self.gear = 0
        self.brake = 0


    def update_state(self, target_idx):
        if target_idx == len(self.path.cx) - 1: # path의 마지막 idx에 도달
            
            states = list(State)
            current_index = states.index(self.state)
            try:
                self.state = states[current_index + 1]  # state update
            except IndexError:
                print("index out of range")
        else:
            pass

    def update_path_and_speed(self, target_idx):
        if self.state.value == "driving":
            self.path.file_open_with_id(self.state.name) #path update
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
        
        elif self.state.value == "parking": # path 생성은 함수 가져오고 speed는 적절한 상수값 설정
            self.path.file_open_with_id(self.state.name) #path update #path를 x,y,yaw,speed나눠서준다(speed는 고정)
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
        
            pass # 범석

        elif self.state.value == "obstacle":
            self.path.file_open_with_id(self.state.name) #path update
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
            pass # 진주

        elif self.state.value == "pickup":
            self.path.file_open_with_id(self.state.name) #path update
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
        
            pass
        
        elif self.state.value == "delivery":
            self.path.file_open_with_id(self.state.name) #path update
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
        
            pass

        elif self.state.value == "traffic_light":
            self.path.file_open_with_id(self.state.name) #path update
            self.set_target_speed(target_idx)
            print(len(self.path.cx) - 1, target_idx, self.state.value, self.state.name)
        
            pass

        else:
            print("error: ", self.state.value)


    def set_target_speed(self, target_idx):
        self.target_speed = self.path.cv[target_idx]


    def cacluate_brake(self): # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= self.adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - self.adapted_speed) / 20.0) * 200
        else:
            brake = 0
        return brake

    def set_gear(self): #parking 상황 더 정교하게 쪼개지면 그에 맞게 후진 상황 수정
        if self.state.value == "parking":
            gear = 0
        else:
            gear = 2
        return gear

    def publish_cmd(self):
        self.steer, target_idx, hdr, ctr = self.st.stanley_control(self.odometry, self.path.cx, self.path.cy, self.path.cyaw, self.reverse)
        self.update_state(target_idx=target_idx)
        self.update_path_and_speed(target_idx=target_idx)
        self.adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=4, max_value=15) # 에러(hdr, ctr) 기반 목표 속력 조정
        self.speed = self.pid.PIDControl(self.odometry.v * 3.6, self.adapted_speed) # speed 조정 (PI control) 
        self.brake = self.cacluate_brake() # brake 조정
        self.gear = self.set_gear() # gear 조정


        msg = ControlMessage()
        msg.speed = self.speed * 10
        msg.steer = int(m.degrees((-1) * self.steer))
        msg.gear = self.gear
        msg.brake = int(self.brake)

        self.pub.publish(msg)



def main():
    rclpy.init(args=None)
    node = rclpy.create_node("state_machine_node")

    # Declare Params
    node.declare_parameter("file_name", "state_machine_test.db")
    node.declare_parameter("odom_topic", "/localization/kinematic_state")

    # Get Params
    file_name = node.get_parameter("file_name").get_parameter_value().string_value
    odom_topic = node.get_parameter("odom_topic").get_parameter_value().string_value



    #Declare Instance
    db = DB(file_name)
    state = State.A1A2
    path = GetPath(db, state)
    odometry = GetOdometry(node, odom_topic)
    state_machine = StateMachine(node, odometry, path, state)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    while rclpy.ok():
        try:
            state_machine.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__ == "__main__":
    main()