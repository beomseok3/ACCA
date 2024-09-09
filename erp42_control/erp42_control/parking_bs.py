import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m
import sys

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import PoseStamped
from tf_transformations import *
from scipy.interpolate import CubicSpline
import sqlite3
import time

from visualization_msgs.msg import Marker, MarkerArray


class PARKING(Node):
    def __init__(self):
        
        super().__init__("parking")
        qos_profile = QoSProfile(depth=10)
        
        #cone
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.callback_cone, qos_profile
        )
        
        #local
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )
        
        self.sub_target_idx = self.create_subscription(
            Int32, "targe_idx", self.callback_targetidx,qos_profile
        )
        
        #parking_path
        self.pub_path = self.create_publisher(Path, "parking_path", qos_profile)

        #gear_publish
        self.pub_gear = self.create_publisher(Int32,"gear", qos_profile)
        #estop_publish
        self.pub_estop = self.create_publisher(Int32,"estop",qos_profile)
        #cone_marker
        self.pub_low_y_cone_marker = self.create_publisher(MarkerArray,"low_y_cone",qos_profile)
        #cone_marker timer
        self.low_y_cone_marker_timer = self.create_timer(1,self.marker_timer)
        #path_timer + state
        self.path_publish = self.create_timer(0.5,self.path)
        #속성
        #marker_id
        self.marker_id = 0
        
        self.low_y_cone = []
        
        
        self.path_x = []
        self.path_y = []
        
        #path_point_step
        self.ds = 0.1
        
        #path
        self.path = []
        
        #domain_idx
        self.idx =0
        
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
        
        # detection_end
        self.detection = 0
        
        # searching(0) start(1) end(2)
        self.state = 0 #default
        
    def path(self):
        if self.detection_end == 1:
            self.calc_path()
        elif self.state != 0:
            if self.state == 1:
                self.gear_publish(0)
            elif self.state == 2:
                self.gear_publish(2)
            path = Path()
            path.header = Header()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"
            for id,x, y, steer in self.path:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, steer)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
            self.pub_path.publish(path)

    def callback_targetidx(self, msg):
        
        if len(self.path) - 15 < msg.data:
            if msg.data >= len(self.path.poses) -5:
                    self.get_logger().info(f"=======goal=======,{self.state}")
                    
                    self.state = 2
                    self.estop_publish(1)
                    time.sleep(5)
                    
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
                                    self.detection_end = 1 # detection_end
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
        db_file = "/home/ps/planning/school_gjs.db"
        conn = sqlite3.connect(db_file)    
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM data")
        rows = cursor.fetchall()
        # print(rows)
        
        for row in rows:
            self.path.append(row)
        conn.close()
        a=self.path[0][1:3]
        b=self.path[-1][1:]
        
        rotate_low_y_cone = self.rotate_points(np.array([self.low_y_cone[self.idx][0] + 1,self.low_y_cone[self.idx][1] - 1.5]),-self.dangle,np.array([self.dlocal_x,self.dlocal_y]))
        dx=rotate_low_y_cone[0]   - a[0]
        dy=rotate_low_y_cone[1] - a[1]
        self.path = np.array(self.path)
        self.path[:,1] += dx
        self.path[:,2] += dy  
        
        
        angle = self.dangle - b[2]
        self.path[:,1:3] = self.rotate_points(self.path[:,1:3],angle,a)
        self.state = 1 # turn state to parking_start
        self.get_logger(f"start : {self.state}")
        
        
        # self.path[:,1:3] = self.rotate_points(self.path[:,1:3],-self.dangle,np.array([self.dlocal_x,self.dlocal_y]))
        # self.path[:,1:3] = self.rotate_points(self.path[:,1:3],-m.pi,np.array([b[0],b[1]]))

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
    
    def gear_publish(self,gear):
        msg = Int32()
        msg.data = gear
        self.pub_gear.publish(msg)
    
    def estop_publish(self,estop):
        msg = Int32()
        msg.data = estop
        self.pub_estop.publish(msg)
        

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