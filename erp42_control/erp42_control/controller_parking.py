import sys
import time
from enum import Enum
import numpy as np
import math as m

# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default

#msg
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray,PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

#tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    sys.path.append("/home/ps/planning/src/state_machine/state_machine")
    from DB import DB
except Exception as e:
    print(e)
    


class PARKING_STATE(Enum):
    SEARCH = 0
    PARKING = 1
    STOP = 2
    RETURN = 3


class POSE:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

# class State:
#     def __init__(self, node, topic):
#         print("cancna")
#         self.node = node
#         self.x = 0.0  # m
#         self.y = 0.0  # m
#         self.yaw = 0.0  # rad
#         self.v = 0.0  # m/s
        
#         self.node.create_subscription(
#             Odometry,
#             topic,
#             callback = self.update,
#             qos_profile= qos_profile_system_default
#         )
        
    def update(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        quaternion = msg.pose.pose.orientation
        _,_, self.yaw = euler_from_quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w)
        
        self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        
class Detection:
    def __init__(self,node,topic):
        self.node = node
        self.low_y_cone = []
        self.node.create_subscription(
            Odometry,
            topic,
            callback = self.update,
            qos_profile = qos_profile_system_default
        )
    def update(self,msg):
        self.cone = msg.poses
    



class CONTROL_PARKING():
    def __init__(self,node):

        self.node = node
        
        # search_path
        self.search_path_db = DB("search_path_kcity.db")
        rows = self.search_path_db.read_db_n("data", "value_x", "value_y", "yaw")
        self.search_path = rows


        # parking_path
        self.parking_path_db = DB("school_gjs.db")
        self.parking_path = self.parking_path_db.read_db_n(
            "data", "value_x", "value_y", "yaw"
        )
        
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        
        # params
        self.standard_point = POSE(
            self.search_path[137][0], self.search_path[137][1], self.search_path[137][2]
        )  
        # kcity : 137 , school : 44
        self.min_x = self.standard_point.x - 10.0
        self.max_x = self.standard_point.x + 20.0
        self.min_y = self.standard_point.y -20.0
        self.max_y = self.standard_point.y +10.0

        self.marker_id = 0
        
        # instance
        # self.state = State(node, "/localization/kinematic_state")
        self.st = Stanley()
        self.detection = Detection(node,"/cone_pose_map")
        
        # parking_state_machine
        self.parking_state = PARKING_STATE.SEARCH
        self.goal = 0
        self.target_idx = 0
        self.stop_start_time = 0
        self.reverse_path = False
        self.low_y_cone = []

        # visualize
        self.low_y_cone_marker_timer = node.create_timer(1, self.marker_timer)
        self.path_timer = node.create_timer(1,self.path_timer)
        self.pub_path = node.create_publisher(Path,"path",qos_profile=10)
        self.pub_low_y_cone_marker = node.create_publisher(
            MarkerArray, "low_y_cone", qos_profile_system_default
        )


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

    def euclidean_duplicate(self, p1):

        threshold = 0.8
        for p2 in self.low_y_cone:
            # print(p2,p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False
    
    def in_detection_area(self, point):

        if self.min_x <= point[0] <= self.max_x:
            if self.min_y <= point[1] <= self.max_y:
                return True
        return False
    # visualize
    def marker_timer(self):
        print("~~~~~~~~~~~")
        if self.low_y_cone:
            marker_array = MarkerArray()
            origin_low_y_cone = Detection.rotate_points(
                np.array(self.low_y_cone),
                -self.standard_point.yaw,
                np.array([self.standard_point.x, self.standard_point.y]),
            )
            for x, y in origin_low_y_cone:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = "cone"
                marker.id = self.marker_id
                self.marker_id += 1  # 고유한 마커 ID 설정
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

                marker_array.markers.append(marker)
            self.pub_low_y_cone_marker.publish(marker_array)
    
    def path_timer(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, yaw in zip(self.path_cx,self.path_cy,self.path_cyaw):
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
        self.pub_path.publish(path)
    
    def detection(self):
        print("detection")
        if self.parking_state == PARKING_STATE.SEARCH:
            for p1 in [(pose.position.x, pose.position.y) for pose in Detection.cone]:
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

        self.parking_path[:, 0] += dx
        self.parking_path[:, 1] += dy
        
        self.parking_path[:, :2] = self.rotate_points(
            self.parking_path[:, :2], angle, self.parking_path[0][:2]
        )  # fix
        
        a = self.search_path_db.find_idx(
            self.parking_path[-1][0], self.parking_path[-1][1], "data"
        )
        self.goal = len(self.search_path) - a - 1
        print(f"{self.goal} is made")

    def control_parking(self,State):
        if self.parking_state == PARKING_STATE.SEARCH:
            self.detection
            
        if self.target_idx >= len(self.path_cx) - self.goal - 1:
            print("change")

            if self.parking_state == PARKING_STATE.SEARCH:
                self.parking_state = PARKING_STATE(self.parking_state.value + 1)  # SEARCH -> PARKING
                self.goal = 0
                self.reverse_path = True
                self.path_cx = self.parking_path[::-1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::-1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::-1, 2]  # 세 번째 열 (cyaw 값들)
                self.target_idx = 0

            elif self.parking_state == PARKING_STATE.PARKING:
                self.parking_state = PARKING_STATE(self.parking_state.value + 1)  # PARKING -> STOP
                self.stop_start_time = time.time()  # STOP 상태로 전환된 시간을 기록
                self.reverse_path = False
                self.path_cx = self.parking_path[:, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[:, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[:, 2]  # 세 번째 열 (cyaw 값들)
                self.target_idx = 0

        else:
            if self.parking_state == PARKING_STATE.STOP:
                if time.time() - self.stop_start_time >= 5.0:
                    self.parking_state = PARKING_STATE(self.parking_state.value + 1)
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
                return msg, False

            else:  # SEARCH, PARKING, RETURN
                if self.parking_state == PARKING_STATE.PARKING:
                    steer, self.target_idx, _, _ = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        reverse=self.reverse_path,
                    )
                    msg = ControlMessage()
                    msg.mora, msg.estop, msg.gear, msg.speed, msg.steer, msg.brake = (
                        0,
                        0,
                        0,
                        3,
                        int(m.degrees(-1 * steer)),  # TO DO: Check reverse /A: cmd msg 다름 m.degrees(-1) = -57.3
                        0,
                    )
                    print(msg)
                    return msg, False
                else:
                    steer, self.target_idx, _, _ = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        reverse=self.reverse_path,
                    )
                    msg = ControlMessage()
                    msg.mora, msg.estop, msg.gear, msg.speed, msg.steer, msg.brake = (
                        0,
                        0,
                        2,
                        3,
                        int(m.degrees(-1*steer)),
                        0,
                    )
                    print(msg)
                    return msg, False

