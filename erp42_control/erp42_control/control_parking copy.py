import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from enum import Enum
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray
import sqlite3
import numpy as np
import math as m
import time
from stanley import Stanley

class ParkingState(Enum):
    IDLE = 0
    DETECTION = 1
    PARKING = 2
    ADJUSTING = 3
    FINISHED = 4

class CONTROL_PARKING(Node):
    def __init__(self):
        super().__init__("control_parking")
        qos_profile = QoSProfile(depth=10)
        
        # State
        self.state = ParkingState.IDLE
        
        # Subscriptions
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.detection_callback, qos_profile
        )
        
        # Stanley controller
        self.st = Stanley()

        # Path
        self.load_path()

        # Internal state
        self.low_y_cone = []
        self.goal_pose = []
        self.start_pose = []
        self.parking_path = []
        self.target_idx = 0
        
        # Command message
        self.gear = 2
        self.brake = 0
        self.estop = 0

    def load_path(self):
        # Load initial path from the database
        db_file = "/home/ps/planning/src/state_machine/db_file/state_machine_path.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT value_x, value_y, yaw FROM data")
        rows = cursor.fetchall()
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        conn.close()


    def parking_control(self, localization):
        if self.state == ParkingState.PARKING:
            steer, target_idx, _, _ = self.st.stanley_control(localization, self.path_cx, self.path_cy, self.path_cyaw, reverse=False)
            
            if len(self.path_cx) == target_idx:
                self.state = ParkingState.ADJUSTING
            
            msg = ControlMessage()
            msg.speed = 5 * 10
            msg.steer = steer
            msg.gear = self.gear
            msg.brake = self.brake
            msg.estop = self.estop
            self.get_logger().info(f"Parking control message: {msg}")
            
            return msg

    def rotate_points(self, points, angle, origin):
        # Rotate points around an origin
        angle_radians = -angle
        rotation_matrix = np.array([
            [np.cos(angle_radians), -np.sin(angle_radians)],
            [np.sin(angle_radians), np.cos(angle_radians)],
        ])
        translated_points = points - origin
        rotated_points = np.dot(translated_points, rotation_matrix.T)
        rotated_points += origin
        return rotated_points

    def euclidean_duplicate(self, p1):
        threshold = 0.8
        for p2 in self.low_y_cone:
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False

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
