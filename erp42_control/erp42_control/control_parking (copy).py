#ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import *
#db
import sys

try:
    sys.path.append("/home/ps/planning/src/state_machine/state_machine")
    
    from DB import DB 
except Exception as e:
    print(e)
#utill
import numpy as np
import math as m
import time
from enum import Enum
#stanley
from stanley import Stanley


class PARKING_STATE(Enum):
    SEARCH = 0
    PARKING = 1
    FINISH =  2

class POSE():
    def __init__(self,x,y,yaw):
        self.x = x
        self.y = y 
        self.yaw = yaw
        
class State():
        def __init__(self, x,y,yaw):

            self.x = x  # m
            self.y = y  # m
            self.yaw = yaw  # rad
            self.v = 0.  # m/s
        def change(self,x,y,yaw):
            self.x = x  # m
            self.y = y  # m
            self.yaw = yaw  # rad

        
        

class CONTROL_PARKING(Node):
    def __init__(self):
        super().__init__("control_parking")
        qos_profile = QoSProfile(depth=10)
        
        #parking_state_machine
        self.state = PARKING_STATE.SEARCH
        self.goal = 0
        self.target_idx = 0
        self.state_machine = self.create_timer(
            0.5,self.state_machine
        )
        #State and localization
        self.local = State(0,0,0)
        self.sub_localization = self.create_subscription(
            Odometry,"localization/kinematic_state", self.calculate_cmd,qos_profile
        )
        
        #cone_pose_map
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.detection_callback, qos_profile
        )
        self.low_y_cone_marker_timer = self.create_timer(1,self.marker_timer)
        
        #stanley
        self.st = Stanley()

        #search_path_initialize
        self.search_path_db = DB("state_machine_path.db")
        rows = self.search_path_db.read_db_n("data","value_x","value_y","yaw")
        self.search_path = rows
        print("\nSEARCH_path_db_is_loaded")
        
        self.pub_low_y_cone_marker = self.create_publisher(MarkerArray,"low_y_cone",qos_profile)
        
        #path_initialize
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        print("\n path_is_loaded")
        
        #parking_path_initialize
        self.parking_path_db = DB("school_gjs.db")
        self.parking_path = self.parking_path_db.read_db_n("data","value_x","value_y","yaw")
        print("\nPARKING_path_db_is_loaded")
        
        #cmd_msgs
        self.cmd = ControlMessage(speed=2*10,steer = 0 ,gear=2, brake=0, estop=0,)
        self.reverse_path = False
        #detection
        self.enough_low_y = 2.0
        self.low_y_cone =[]
        self.standard_point = POSE(self.search_path[0][0],self.search_path[0][1],self.search_path[0][2])
        #define_detection_area
        self.min_x = self.standard_point.x
        self.max_x = self.standard_point.x + 20.0
        self.min_y = self.standard_point.y - 1.5
        self.max_y = self.standard_point.y 
        
        self.marker_id =0
        self.i = 0
        self.flag =0
        

    #add
    def marker_timer(self):
 
        if self.low_y_cone:
            marker_array = MarkerArray()
            origin_low_y_cone = self.rotate_points(np.array(self.low_y_cone),-self.standard_point.yaw,np.array([self.standard_point.x,self.standard_point.y]))
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
        
    def in_detection_area(self, point):
        
        if self.min_x <= point[0] <= self.max_x:
           if self.min_y <= point[1] <= self.max_y:
               return True
        return False
        
    def rotate_points(self, points, angle, origin):
        if points is not None:
            
            angle_radians = -angle # 반 시계방향
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
        
    def euclidean_duplicate(self, p1):
        
        threshold = 0.8
        for p2 in self.low_y_cone:
            # print(p2,p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False

    def update_parking_path_if_needed(self):
        
        for i in range(len(self.low_y_cone) - 1):
            # print(self.low_y_cone)
            dist = self.low_y_cone[i+1][0] - self.low_y_cone[i][0]
            print(dist)
            if dist > 3.0:
                self.idx = i
                self.get_logger().info("ododododoo")
                self.adjust_parking_path()
                break


    def adjust_parking_path(self):
        
        goal_pose_c = self.rotate_points(
            np.array([self.low_y_cone[self.idx][0] + 1, self.low_y_cone[self.idx][1] - 1.5]), -self.standard_point.yaw, np.array([self.standard_point.x,self.standard_point.y])
            )
        dx = goal_pose_c[0] - self.parking_path[0][0]
        dy = goal_pose_c[1] - self.parking_path[0][1]
        
        self.parking_path = np.array(self.parking_path)
        self.parking_path[:,0] += dx
        self.parking_path[:,1] += dy
        
        angle = self.standard_point.yaw - self.parking_path[0][2]
        self.parking_path[:,:2] = self.rotate_points(self.parking_path[:,:2], angle, self.parking_path[0][:2]) #fix
        a= self.search_path_db.find_idx(self.parking_path[0][0],self.parking_path[0][1],"data")
        self.goal = len(self.path_cx) - a -1
        self.get_logger().info(f"{self.goal} is made")
        # self.get_logger().info(f"{self.goal}")

    def detection(self, msg):
        if self.state == PARKING_STATE.SEARCH:
            for p1 in [(pose.position.x, pose.position.y) for pose in msg.poses]:
                rotated_p1 = self.rotate_points(np.array([p1]), self.standard_point.yaw, np.array([self.standard_point.x, self.standard_point.y]))
                rotated_p1 = tuple(rotated_p1[0])
                if not self.euclidean_duplicate(rotated_p1):
                    if self.in_detection_area(rotated_p1):
                        self.low_y_cone.append(rotated_p1)
                        self.update_parking_path_if_needed()
                    else:
                        pass
                        # self.get_logger().info(f"Out of detection area: {p1}")
                else:
                    pass
                    # self.get_logger().info(f"{p1} is duplicate")
        
        
        
        
    # main    
    def detection_callback(self, msg):
        
        if self.state == PARKING_STATE.SEARCH:
            self.detection(msg)

    def state_machine(self):
        if self.target_idx >= len(self.path_cx) - self.goal - 1:
            if self.state == PARKING_STATE.SEARCH:
                self.goal = 0
                self.cmd.gear = 0
                self.reverse_path = True
                print(self.parking_path)
                self.path_cx = self.parking_path[::-1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::-1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::-1, 2]  # 세 번째 열 (cyaw 값들)

                    
            elif self.state == PARKING_STATE.PARKING:
                current_time = time.time()
                self.reverse_path= False
                self.cmd.gear=2
                self.cmd.estop = 1
                self.path_cx = self.parking_path[:, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[:, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[:, 2]  # 세 번째 열 (cyaw 값들)
                

                    
                
            self.state = PARKING_STATE(self.state.value + 1)
        
            
            self.get_logger().info(f"State changed to {self.state.name}.")
            
        else:
            # self.get_logger().info(f"Continuing in state: {self.state.name}.")
            if self.cmd.estop == 1:
                
            # pass
            
    def calculate_cmd(self, msg):
        if self.flag == 1:
            self.flag == 0
            time.sleep(5)
            self.cmd.estop = 0
        if self.cmd.estop == 1:
            self.flag = 1
            print(self.cmd)
        print(self.target_idx, len(self.path_cx)-1, self.goal)
        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.local.change(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
        steer,self.target_idx,_,_ = self.st.stanley_control(self.local,self.path_cx,self.path_cy,self.path_cyaw,last_target_idx=0,reverse=self.reverse_path)
        # print(steer)
        self.cmd.steer = int(m.degrees((-1)*steer))
        # print(self.cmd)
        
        return(self.state,self.cmd)
    
    

    

def main(args=None):
    rclpy.init(args=args)
    node = CONTROL_PARKING()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
