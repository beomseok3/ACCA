import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from scipy.interpolate import CubicSpline
from scipy.cluster.hierarchy import linkage, fcluster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from shapely.geometry import Point, Polygon
from stanley import Stanley
from erp42_msgs.msg import SerialFeedBack, ControlMessage


class Obstacle():
    def __init__(self, node):

        self.node = node
        self.sub_marker = self.node.create_subscription(
            MarkerArray, "/markers", self.call_marker, 10
        )  # 장애물 위치
        # Publishers
        self.ref_path1 = self.node.create_publisher(Path, "/ref/path1", 10)  # 1차선
        self.ref_path2 = self.node.create_publisher(Path, "/ref/path2", 10)  # 2차선

        self.LocalPath_pub = self.node.create_publisher(
            Path, "/path/avoid_path", 10
        )  # 로컬 패스

        self.marker_pub = self.node.create_publisher(
            MarkerArray, "transformed_markers", 10
        )  # 곧 지울꺼
        self.pub = self.node.create_publisher(ControlMessage, "/cmd_msg", 10)
        # self.timer = self.node.create_timer(0.1, self.timer_callback)

        # 현재 위치

        self.odometry = None

        self.yaw = None

        self.ori_x = None
        self.ori_y = None
        self.ori_x = None
        self.ori_w = None

        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        # 장애물 -> points_cloud
        self.obs = np.array([]).reshape(0, 2)
        self.near_obstacle = False
        self.num1_obs = []
        self.num2_obs = []
        self.obs_vector = None

        self.to_num = None
        # 도로 center_points #현재는 직접하는데 나중에 받아와야함
        self.ref_path_points1 = None
        self.ref_path_points2 = None

        # local_path
        self.local_points = None  # list

        self.local_x = []
        self.local_y = []
        self.local_yaw = []
        self.v = 0
        # stanley ?이거 맞나
        self.st = Stanley()

    def call_marker(self, msg):
        if self.odometry is not None:
            markers = msg.markers
            transformed_marker_array = MarkerArray()
            observed_points = []

            for p in markers:
                points = np.array([[point.x, point.y] for point in p.points])
                if len(points) > 0:
                    # points의 중점을 계산
                    center = np.min(points, axis=0)

                    transformed_center = self.transform_cluster_centers(
                        np.array([center])
                    )

                    # 변환된 중심점을 저장
                    observed_points.append(transformed_center[0])

                    # 시각화용 마커 생성
                    transformed_marker = Marker()
                    transformed_marker.header.frame_id = "map"
                    transformed_marker.header.stamp = self.node.get_clock().now().to_msg()
                    transformed_marker.ns = "transformed_marker"
                    transformed_marker.id = p.id
                    transformed_marker.type = Marker.SPHERE
                    transformed_marker.action = Marker.ADD
                    transformed_marker.pose.position.x = transformed_center[0, 0]
                    transformed_marker.pose.position.y = transformed_center[0, 1]
                    transformed_marker.pose.position.z = 0.0
                    transformed_marker.scale.x = 0.5
                    transformed_marker.scale.y = 0.5
                    transformed_marker.scale.z = 0.5
                    transformed_marker.color.a = 1.0
                    transformed_marker.color.r = 0.0
                    transformed_marker.color.g = 1.0
                    transformed_marker.color.b = 0.0

                    transformed_marker_array.markers.append(transformed_marker)

            # 변환된 중심점들을 numpy 배열로 저장
            if observed_points:
                self.obs = np.array(observed_points)

            # 변환된 마커 퍼블리시
            self.marker_pub.publish(transformed_marker_array)

    def rotate_points(self, points, angle):
        angle_radians = np.deg2rad(angle)
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        rotated_points = np.dot(points, rotation_matrix)
        return rotated_points

    def transform_cluster_centers(self, cluster_centers):
        if len(cluster_centers) == 0:
            return np.array([])
        euler = euler_from_quaternion(self.odom_orientation)
        _, _, yaw = euler

        rotation_angle = np.rad2deg(-yaw)

        rotated_centers = self.rotate_points(cluster_centers, rotation_angle)

        transformed_centers = rotated_centers + np.array(
            (self.odometry.x, self.odometry.y)
        )

        return transformed_centers

    def publish_ref_path(self, wx, wy, num=None):
        # Interpolate y values given x using CubicSpline
        cs_x = CubicSpline(range(len(wx)), wx)
        cs_y = CubicSpline(range(len(wy)), wy)

        # Calculate total path length based on the given waypoints
        distances = np.sqrt(np.diff(wx) ** 2 + np.diff(wy) ** 2)
        total_length = np.sum(distances)

        # 간격
        sampling = 0.1
        # Sampling intervals for generating the path with 0.1 distance between points
        s = np.arange(0, len(wx) - 1, sampling / total_length * (len(wx) - 1))

        # Interpolated path coordinates
        rx = cs_x(s)
        ry = cs_y(s)

        # Save path and direction data
        path_points = np.vstack((rx, ry)).T

        # if num == 1:
        #     if self.global_path_points1 is not None:
        #         path_points = self.global_path_points1
        # if num == 2:
        #     if self.global_path_points2 is not None:
        #         path_points = self.global_path_points2

        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Compute orientation from current position
            yaw = np.arctan2(y - self.odometry.y, x - self.odometry.x)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)

        if num == 1:
            # Publish the global path
            self.ref_path1.publish(path)

            # Save global path for later use
            self.ref_path_points1 = path_points

        if num == 2:
            # Publish the global path
            self.ref_path2.publish(path)

            # Save global path for later use
            self.ref_path_points2 = path_points

    def publish_local_path(self, points):
        # Initialize lists to store x, y, and yaw values
        local_x = []
        local_y = []
        local_yaw = []

        # Extract x and y coordinates from points
        for x_points, y_points in points:
            local_x.append(x_points)
            local_y.append(y_points)

        # Calculate distances between consecutive points
        dx = np.diff(local_x)
        dy = np.diff(local_y)
        distances = np.sqrt(dx**2 + dy**2)

        # Interpolation to achieve 0.1 spacing
        total_distance = np.cumsum(distances)
        total_distance = np.insert(total_distance, 0, 0)  # Insert 0 at the beginning
        interp_distances = np.arange(
            0, total_distance[-1], 0.1
        )  # Interpolated distances

        # Interpolate x and y coordinates
        interp_x = np.interp(interp_distances, total_distance, local_x)
        interp_y = np.interp(interp_distances, total_distance, local_y)

        path_points = []

        path = Path()
        path.header.frame_id = "map"

        for i in range(len(interp_x) - 1):
            x = interp_x[i]
            y = interp_y[i]
            next_x = interp_x[i + 1]
            next_y = interp_y[i + 1]

            # Calculate yaw from the direction of the next point
            yaw = np.arctan2(next_y - y, next_x - x)
            local_yaw.append(yaw)

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0  # Set z-coordinate

            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

            path_points.append((x, y))

        self.LocalPath_pub.publish(path)

        # Store the interpolated path points and yaw
        self.local_x = interp_x.tolist()
        self.local_y = interp_y.tolist()
        self.local_yaw = local_yaw

    def line_change(self):

        for obs_x, obs_y in self.num1_obs:
            _, _, yaw = euler_from_quaternion(self.odom_orientation)

            # Compute the direction vector of the vehicle
            vehicle_direction = np.array([cos(yaw), sin(yaw)])
            # odom과 self.num1_obs 비교후 odom보다 앞에있으면 to_num = 2
            self.obs_vector = np.array(
                [obs_x - self.odometry.x, obs_y - self.odometry.y]
            )

        for obs_x, obs_y in self.num2_obs:
            _, _, yaw = euler_from_quaternion(self.odom_orientation)

            vehicle_direction = np.array([cos(yaw), sin(yaw)])

            obs_vector = np.array([obs_x - self.odometry.x, obs_y - self.odometry.y])
            obs_length = sqrt(
                (obs_x - self.odometry.x) ** 2 + (obs_y - self.odometry.y) ** 2
            )

            if (
                np.dot(vehicle_direction, obs_vector) > 0
                and np.dot(vehicle_direction, self.obs_vector) < 0
                and obs_length < 5
            ):

                self.to_num = 1
                self.local_points = self.ref_path_points1.tolist()
                self.publish_local_path(self.local_points)
                return

        if self.to_num is None:
            self.local_points = self.ref_path_points2.tolist()
            self.to_num = 2
            print("s")

        self.publish_local_path(self.local_points)

    def check_obstacle(self, size_type):
        if size_type == "small":

            line1_DA = [
                (28.821, 104.308),
                (29.4677, 106.568),
                (33.539, 104.759),
                (32.6035, 102.745),
            ]  # DA = detection_area

            line2_DA = [
                (30.2457, 106.764),
                (31.1524, 108.339),
                (26.759, 110.279),
                (25.9166, 108.797),
            ]
        elif size_type == "big":

            line1_DA = [
                (125.193, 249.699),
                (129.132, 247.489),
                (132.83, 256.292),
                (127.887, 258.721),
            ]  # DA = detection_area

            line2_DA = [
                (133.471, 259.038),
                (137.802, 257.581),
                (140.81, 266.478),
                (135.475, 268.154),
            ]

        # Create polygons from the detection areas
        polygon1 = Polygon(line1_DA)
        polygon2 = Polygon(line2_DA)

        # Check if any of the obstacles are within the detection area
        for obs_point in self.obs:
            point = Point(obs_point)
            if polygon1.contains(point):

                self.num1_obs.append((point.x, point.y))
                break

        # Check if any of the obstacles are within the detection area
        for obs_point in self.obs:
            point = Point(obs_point)
            if polygon2.contains(point):

                self.num2_obs.append((point.x, point.y))
                break

    def control_obstacle(self, odometry):
        self.odometry = odometry

        self.timer_callback(odometry)
        msg = ControlMessage()

        steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            odometry,
            self.local_x,
            self.local_y,
            self.local_yaw,
        )

        msg.speed = 10
        msg.steer = int(degrees((-1) * steer))
        msg.gear = 0

        # print(msg.steer)
        # print(self.target_idx)
        self.pub.publish(msg)
        if self.target_idx >= len(self.local_points) - 30:
            print("True")

            self.to_num = None
            self.num1_obs = []
            self.num2_obs = []
            return msg, True
        else:

            return msg, False

    def timer_callback(self, odometry):

        print(self.to_num)
        self.odom_pose = np.array([self.odometry.x, self.odometry.y])
        q = quaternion_from_euler(0, 0, self.odometry.yaw)
        self.odom_orientation = [q[0], q[1], q[2], q[3]]

        if self.odometry.x is not None:

            if sqrt(
                (self.odometry.x - 39.0068) ** 2 + (self.odometry.y - 100.236) ** 2
            ) < sqrt(
                (self.odometry.x - 122.801) ** 2 + (self.odometry.y - 238.011) ** 2
            ):
                # k-city2 @작은거

                self.publish_ref_path(
                    wx=[
                        39.0068,
                        23.4645,
                    ],
                    wy=[
                        100.236,
                        108.442,
                    ],
                    num=1,
                )  # 1차선 center line

                self.publish_ref_path(
                    wx=[
                        38.5194,
                        19.4975,
                    ],
                    wy=[
                        103.62,
                        112.543,
                    ],
                    num=2,
                )  # 2차선 center line
                self.check_obstacle("small")

            else:

                # k-city2 @큰거
                self.to_num = None
                self.publish_ref_path(
                    wx=[
                        121.565,
                        135.176,
                    ],
                    wy=[
                        234.307,
                        277.412,
                    ],
                    num=1,
                )  # 1차선 center line

                self.publish_ref_path(
                    wx=[
                        128.593,
                        138.154,
                    ],
                    wy=[
                        239.503,
                        270.844,
                    ],
                    num=2,
                )  # 2차선 center line
                self.check_obstacle("big")

            self.line_change()

