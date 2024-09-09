#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import time as t

left_points = []
right_points = [] 


class BevierCurveMaking(Node):

    def __init__(self):
        super().__init__('bevier_curve_maker')
        qos_profile = QoSProfile(depth=10)

        self.sub_left = self.create_subscription(
            Float32MultiArray,
            'blue', 
            self.left_points_callback,
            qos_profile)
        
        self.sub_right = self.create_subscription(
            Float32MultiArray,
            'yellow',
            self.right_points_callback,
            qos_profile)

        self.pub_path = self.create_publisher(
            Path,
            '/bezier_curve_path_making',
            qos_profile)

        #self.timer = self.create_timer(0.5, self.timer_callback)

    def left_points_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 3)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        
        for i in range (len(x)):
            
            self.update_points(x[i], y[i], z[i], 'left')

    def right_points_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 3)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        
        for i in range (len(x)):
            
            self.update_points(x[i], y[i], z[i], 'right')

    def update_points(self, x, y, z, vector):
        if vector == 'left':
            left_points.append((x, y, z))
        elif vector == 'right':
            right_points.append((x, y, z))

        self.point_select()

    def point_select(self):
        print("Left Points:", left_points)
        print("Right Points:", right_points)

        lenth = min(len(left_points), len(right_points))

        p0 = (0, 0)  # Current position
        p1 = left_points[0] if lenth else (0, 0)
        p2 = right_points[0] if lenth else (0, 0)
        p3 = left_points[1] if lenth > 1 else (0, 0)
        p4 = right_points[1] if lenth > 1 else (0, 0)
        p5 = left_points[2] if lenth > 2 else (0, 0)
        p6 = right_points[2] if lenth > 2 else (0, 0)

        print("Left points for bezier:", p0, p1, p3, p5)
        print("Right points for bezier:", p0, p2, p4, p6)

        if lenth > 0:
            left_bezier_x, left_bezier_y = self.quad_bez(p0, p1, p3, p5)
            right_bezier_x, right_bezier_y = self.quad_bez(p0, p2, p4, p6)

            mid_bezier_x, mid_bezier_y = (left_bezier_x + right_bezier_x) / 2, (left_bezier_y + right_bezier_y) / 2

            dx = np.diff(mid_bezier_x)
            dy = np.diff(mid_bezier_y)

            self.publish_bevier(mid_bezier_x, mid_bezier_y, dx, dy)

    def quad_bez(self, p0, p1, p2, p3, t=100):
        t1 = np.linspace(0, 1, t)
        if p3 != (0, 0):
            b_x = ((1 - t1) ** 3 * p0[0] +
                   3 * (1 - t1) ** 2 * t1 * p1[0] +
                   3 * (1 - t1) ** 1 * t1 ** 2 * p2[0] +
                   t1 ** 3 * p3[0])
            b_y = ((1 - t1) ** 3 * p0[1] +
                   3 * (1 - t1) ** 2 * t1 * p1[1] +
                   3 * (1 - t1) ** 1 * t1 ** 2 * p2[1] +
                   t1 ** 3 * p3[1])
            return b_x, b_y
        elif p2 != (0, 0):
            b_x = ((1 - t1) ** 2 * p0[0] +
                   2 * (1 - t1) * t1 * p1[0] +
                   t1 ** 2 * p2[0])
            b_y = ((1 - t1) ** 2 * p0[1] +
                   2 * (1 - t1) * t1 * p1[1] +
                   t1 ** 2 * p2[1])
            return b_x, b_y
        elif p1 != (0, 0):
            b_x = ((1 - t1) * p0[0] +
                   t1 * p1[0])
            b_y = ((1 - t1) * p0[1] +
                   t1 * p1[1])
            return b_x, b_y
        else:
            print("Not enough points to calculate bezier curve")

    def publish_bevier(self, mid_bezier_x, mid_bezier_y, dx, dy):
        path = Path()
        path.header.frame_id = 'velodyne'

        for x, y, dx, dy in zip(mid_bezier_x, mid_bezier_y, dx, dy):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0
            
            yaw = np.arctan2(dy, dx)
            q = self.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        self.pub_path.publish(path)
        left_points.clear()
        right_points.clear()


    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        self.point_select()


def main(args=None):
    rclpy.init(args=args)
    bevier_curve_maker = BevierCurveMaking()
    rclpy.spin(bevier_curve_maker)
    bevier_curve_maker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()