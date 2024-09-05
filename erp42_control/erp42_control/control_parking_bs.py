import sys 
from enum import Enum

# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,qos_profile_system_default

#msg
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray,PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

#tf 
from tf_transformations import *
from tf2_ros import Buffer, TransformStamped

# db
try:
    sys.path.append("/home/ps/planning/src/state_machine/state_machine")
    from DB import DB
except Exception as e:
    print(e)
    
# utill
import numpy as np
import math as m
import time

# stanley
from stanley import Stanley


class Parkingstate(Enum):
    SEARCH = 0
    PARKING = 1
    STOP = 2
    RETURN = 3


class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class Cone:
    def __init__(self,node,topic,search_path):
        self.node = node 
        
        self.node.create_subscription(
            PoseArray,
            topic,
            callback = self.detection,
            qos_profile = qos_profile_system_default
        )
        
        #param
        
        self.standard_point = Pose(
            search_path[44][0], search_path[44][1], search_path[44][2]
        )  
        # kcity : 137 , school : 44
        self.low_y_cone = []
        self.min_x = self.standard_point.x - 1.0
        self.max_x = self.standard_point.x + 20.0
        self.min_y = self.standard_point.y -2.0
        self.max_y = self.standard_point.y 
        

    def detection(self,msg):
        if self.parking_state == Parkingstate.SEARCH:
            for p1 in [(pose.position.x, pose.position.y) for pose in msg.poses]:
                rotated_p1 = self.rotate_points(
                    np.array([p1]),
                    self.standard_point.yaw,
                    np.array([self.standard_point.x, self.standard_point.y]),
                )
                rotated_p1 = tuple(rotated_p1[0])
                if not self.euclidean_duplicate(rotated_p1):
                    if self.in_detection_area(rotated_p1):
                        self.low_y_cone.append(rotated_p1)
                        self.update_parking_path()
    
    def in_detection_area(self,point):
        if self.min_x <= point[0] <= self.max_x:
            if self.min_y <= point[1] <= self.max_y:
                return True
        return False
    
    def update_parking_path(self):
        for i in range(len(self.low_y_cone) - 1):
            # print(self.low_y_cone)
            dist = self.low_y_cone[i + 1][0] - self.low_y_cone[i][0]
            print(dist)
            if dist > 4.0:
                self.idx = i
                self.adjust_parking_path()
                break
    def adjust_parking_path(self):

        angle = self.standard_point.yaw - self.parking_path[-1][2]
        
        goal_pose_c = self.rotate_points(
            np.array(
                [self.low_y_cone[self.idx][0] + 1, self.low_y_cone[self.idx][1] - 1.5]
            ),
            -self.standard_point.yaw,
            np.array([self.standard_point.x, self.standard_point.y]),
        )

        dx = goal_pose_c[0] - self.parking_path[0][0]
        dy = goal_pose_c[1] - self.parking_path[0][1]

        self.parking_path = np.array(self.parking_path)
        self.parking_path[:, 0] += dx
        self.parking_path[:, 1] += dy
        
        self.parking_path[:, :2] = self.rotate_points(
            self.parking_path[:, :2], angle, self.parking_path[0][:2]
        )  # fix
        
        a = self.search_path_db.find_idx(
            self.parking_path[-1][0], self.parking_path[-1][1], "data"
        )
        print(a)
        self.goal = len(self.search_path) - a - 1
        self.get_logger().info(f"{self.goal} is made")
    
    def rotate_points(self, points, angle, origin):
        if points is not None:

            angle_radians = -angle  # 반 시계방향
            rotation_matrix = np.array(
                [
                    [np.cos(angle_radians), -np.sin(angle_radians)],
                    [np.sin(angle_radians), np.cos(angle_radians)],
                ]
            )

            translated_points = points - origin
            rotated_points = np.dot(translated_points, rotation_matrix.T)
            rotated_points += origin
            return rotated_points
        else:
            pass
    
    def euclidean_duplicate(self,p1):
        threshold = 0.8
        for p2 in self.low_y_cone:
            # print(p2,p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False

        

class State: # Node를 부모 state machine에서 줄 수 있다.
    def __init__(self, node, topic):
        self.node = node
        
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.yaw = 0.0  # rad
        self.v = 0.0  # m/s
        
        self.node.create_subscription(
            Odometry,
            topic,
            callback = self.update,
            qosprofile = qos_profile_system_default
        )
        
    def update(self,msg):
        self.x = msg.pose.pose.position.x
        self.x = msg.pose.pose.position.x
        
        quaternion = msg.pose.pose.orientation
        _,_, self.yaw = euler_from_quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w)
        
        self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

class Control_parking(Node):
    def __init__(self):
        super().__init__("control_parking")
        
        #DB
        self.search_path_db = DB("search_path_school.db")
        rows = self.search_path_db.read_db_n("data", "value_x", "value_y", "yaw")
        self.search_path = rows
        
        self.parking_path_db = DB("school_gjs.db")
        self.parking_path = self.parking_path_db.read_db_n(
            "data", "value_x", "value_y", "yaw"
        )
        
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        
        # instance
        self.state = State(self, "/localization/kinematic_state")
        self.stanley = Stanley()
        self.cone = Cone(self,"cone_pose_map",self.search_path)

        # state_machine
        self.parking_state = Parkingstate.SEARCH
        self.goal = 0
        self.target_idx = 0
        self.reverse_path = False
        self.stop_start_time = 0

    # call_func
    def control_parking(self, odometry):
        # print(self.state, self.target_idx, len(self.path_cx))
        
        if self.target_idx >= len(self.path_cx) - self.goal - 1:

            if self.state == Parkingstate.SEARCH:
                self.state = Parkingstate(self.state.value + 1)  # SEARCH -> PARKING
                self.goal = 0
                self.reverse_path = True
                self.path_cx = self.parking_path[::-1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::-1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::-1, 2]  # 세 번째 열 (cyaw 값들)
                self.target_idx = 0

            elif self.state == Parkingstate.PARKING:
                self.state = Parkingstate(self.state.value + 1)  # PARKING -> STOP
                self.stop_start_time = time.time()  # STOP 상태로 전환된 시간을 기록
                self.reverse_path = False
                self.path_cx = self.parking_path[:, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[:, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[:, 2]  # 세 번째 열 (cyaw 값들)
                self.target_idx = 0

        else:
            if self.state == Parkingstate.STOP:
                if time.time() - self.stop_start_time >= 5.0:
                    self.state = Parkingstate(self.state.value + 1)
                    self.target_idx = 0  # STOP -> RETURN

                msg = ControlMessage()
                msg.mora, msg.estop, msg.gear, msg.speed, msg.steer, msg.brake = (
                    0,
                    1,
                    0,
                    0,
                    0,
                    200,
                )
                print(msg)
                return self.state,msg

            else:  # SEARCH, PARKING, RETURN
                if self.state == Parkingstate.PARKING:
                    steer, self.target_idx, _, _ = self.st.stanley_control(
                        odometry,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        last_target_idx=0,
                        reverse=self.reverse_path,
                    )
                    msg = ControlMessage()
                    msg.mora, msg.estop, msg.gear, msg.speed, msg.steer, msg.brake = (
                        0,
                        0,
                        0,
                        30,
                        int(m.degrees(-1) * steer),  # TO DO: Check reverse /A: cmd msg 다름 m.degrees(-1) = -57.3
                        0,
                    )
                    print(msg)
                    return self.state,msg
                else:
                    steer, self.target_idx, _, _ = self.st.stanley_control(
                        odometry,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        last_target_idx=0,
                        reverse=self.reverse_path,
                    )
                    msg = ControlMessage()
                    msg.mora, msg.estop, msg.gear, msg.speed, msg.steer, msg.brake = (
                        0,
                        0,
                        2,
                        30,
                        int(m.degrees(-1)*steer),
                        0,
                    )
                    print(msg)
                    return self.state,msg
