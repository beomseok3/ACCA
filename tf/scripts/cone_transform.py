#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as TFPose


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.cone_pose_sub = self.create_subscription(PoseArray, "/cone_poses", callback=self.callback, qos_profile=qos_profile_system_default)
        self.pub = self.create_publisher(PoseArray, "/cone_pose_map", qos_profile=qos_profile_system_default)

        self.tf_buffer = Buffer(Duration(seconds=1.0), self)
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos_profile_system_default)


    def callback(self, msg):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header = Header(frame_id="map", stamp=Time().to_msg())

        if self.tf_buffer.can_transform("map", "velodyne", Time().to_msg(), Duration(seconds=1.0)):
            print("HELLO WORLD!")
            return
            for pose in msg.poses:
                tf_pose = TFPose()
                tf_pose.header.frame_id = "velodyne"
                tf_pose.header.stamp = Time().to_msg()

                tf_pose.pose.position = pose.position
                tf_pose.pose.orientation = pose.orientation


                transformed_pose = self.tf_buffer.transform(tf_pose, "map", Duration(seconds=1.0))
                new_pose = Pose()
                new_pose.position = transformed_pose.pose.position
                new_pose.orientation = transformed_pose.pose.orientation
                transformed_pose_array.poses.append(new_pose)

            print(len(transformed_pose_array.poses))
        else:
            self.get_logger().warn("Cannot lookup transform")


        self.pub.publish(transformed_pose_array)



def main():
    rclpy.init(args=None)

    node = TestNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()