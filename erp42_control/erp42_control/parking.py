import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m
import sys

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Int32
from tf_transformations import *
import sqlite3
import time

from visualization_msgs.msg import Marker, MarkerArray



class PARKING(Node):
    def __init__(self):
        
        super().__init__("parking")
        qos_profile = QoSProfile(depth=10)
        
        #node_fuction
        #cone_pose_map
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.callback_cone, qos_profile
        )
        
        # localization
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )
        #parking_path
        self.pub_path = self.create_publisher(Path, "parking_path", qos_profile)
        #close_cone
        self.pub_low_y_cone_marker = self.create_publisher(MarkerArray,"low_y_cone",qos_profile)
        #cone_marker timer
        self.low_y_cone_marker_timer = self.create_timer(1,self.marker_timer)
        #path_timer + state_machine
        self.path_publish = self.create_timer(0.5,self.path)
       
        #var
        #marker_id
        self.marker_id = 0
        
        #close_cone
        self.low_y_cone = []
        
        #return_path
        self.path_x = []
        self.path_y = []
        
        #path_point_step
        self.ds = 0.1
        
        #path
        self.path = []
        
        #detection_area
        self.min_x = 0.0
        self.min_y = 0.0
        self.max_x = 0.0
        self.max_y = 0.0
        
        # verify_low_y_cone
        self.enough_low_y = 2.0
        
        # static_local
        self.dangle = 0
        self.dlocal_x = 0
        self.dlocal_y = 0
        
        # update_always
        self.local_x = 0.0
        self.local_y = 0.0
        self.angle = 0
        
    
    

    def euclidean_duplicate(self, p1):
        threshold = 0.8
        for p2 in self.low_y_cone:
            # print(p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False
            
    def callback_cone(self, msg):
        if not self.detection:
            for p1 in [
                (pose.position.x, pose.position.y) for pose in msg.poses
            ]:
                if not self.low_y_cone:
                    
                    p1_rotated = self.rotate_points(np.array([p1]),self.angle,np.array([self.local_x,self.local_y]))
                    p1_rotated = tuple(p1_rotated[0])
                    
                    y_dis = abs(p1_rotated[1] - self.local_y)
                    if y_dis <= self.enough_low_y:
                        self.low_y_cone.append(p1_rotated)
                        
                        # reserve local
                        self.dangle = self.angle
                        self.dlocal_x = self.local_x
                        self.dlocal_y = self.local_y
                        
                        #detection_area
                        self.min_x = p1_rotated[0] 
                        self.max_x = p1_rotated[0] + 16
                        self.min_y = p1_rotated[1] - 1.0
                        self.max_y = p1_rotated[1] + 0.8
                            
                    else:
                        continue
                else:
                    
                    rotated_p1 = self.rotate_points(np.array([p1]), self.dangle, np.array([self.dlocal_x, self.dlocal_y]))
                    rotated_p1 = tuple(rotated_p1[0])
                    # print(rotated_p1)
                    if not self.euclidean_duplicate(rotated_p1):
                        if self.min_x <= rotated_p1[0] <= self.max_x and self.min_y <= rotated_p1[1] <= self.max_y:
                            self.low_y_cone.append(rotated_p1)
                            for i in range(len(self.low_y_cone) - 1):
                                dist = self.low_y_cone[i+1][0] - self. low_y_cone[i][0]
                                if dist > 4.0:
                                    self.calc_path()
                                    self.idx = i # use i and i+1 for path_planning
                                    break
                        else:
                            continue
                    else:
                        continue
                
    def callback_local(self, msg):
        self.local_x = msg.pose.pose.position.x
        self.local_y = msg.pose.pose.position.y
        quarternion = [
            0,
            0,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.angle = euler_from_quaternion(quarternion)
        
    
    def marker_timer(self):
 
        if self.low_y_cone:
            marker_array = MarkerArray()
            origin_low_y_cone = self.rotate_points(np.array(self.low_y_cone),-self.dangle,np.array([self.dlocal_x,self.dlocal_y]))
            for x, y in origin_low_y_cone:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "cone"
                marker.id = self.marker_id 
                self.marker_id += 1# 고유한 마커 ID 설정
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
                marker.lifetime = rclpy.duration.Duration(
                    seconds=0
                ).to_msg()

                marker_array.markers.append(marker)

            self.pub_low_y_cone_marker.publish(marker_array)
        
    #utill    
    def calc_path(self):
        #path 가져오기
        if True:# detection done
            db_file = "/home/ps/planning/school_gjs.db"
            conn = sqlite3.connect(db_file)
                
            cursor = conn.cursor()
            
            cursor.execute("SELECT x_value,y_value,yaw FROM data")
            rows = cursor.fetchall()
            
            for row in rows:
                self.path.append(row)
            conn.close()
            
            #goal
            a=self.path[0][0:2]
            #start
            b=self.path[-1][:]
            
            rotate_low_y_cone = self.rotate_points(np.array([self.low_y_cone[self.idx][0] + 1,self.low_y_cone[self.idx][1] - 1.5]),-self.dangle,np.array([self.dlocal_x,self.dlocal_y]))
            
            #translation
            dx=rotate_low_y_cone[0]   - a[0]
            dy=rotate_low_y_cone[1] - a[1]
            self.path = np.array(self.path)
            self.path[:,0] += dx
            self.path[:,1] += dy  
            #rotation
            angle = self.dangle - b[2]
            self.path[:,1:3] = self.rotate_points(self.path[:,1:3],angle,a)
            
            #idx_inverse
            # self.path[:] = self.path[::-1]
            # print(self.path)
            
            #logger
            self.get_logger().info(f"Path calculation done: {self.path}")
        else:
            return global_path

    #utill        
    def rotate_points(self, points, angle, origin):
        angle_radians = -angle # 반 시계방향
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        
        # 원점으로 평행이동
        translated_points = points - origin
        # 각 만큼 회전
        rotated_points = np.dot(translated_points, rotation_matrix.T)
        # 회전 중심 위치로 평행이동
        rotated_points += origin
        
        return rotated_points
    

def main(args=sys.argv):
    rclpy.init(args=args)
    # print(f"{args}")
    node = PARKING()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()