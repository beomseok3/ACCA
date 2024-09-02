#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ConePoseTransformer : public rclcpp::Node
{
public:
    ConePoseTransformer()
        : Node("cone_pose_transformer_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        cone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/cone_poses", 10, 
            std::bind(&ConePoseTransformer::cone_pose_callback, this, std::placeholders::_1));

        cone_pose_map_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose_map", 10);
    }

private:
    void cone_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        geometry_msgs::msg::PoseArray transformed_poses;
        transformed_poses.header.frame_id = "map";
        transformed_poses.header.stamp = msg->header.stamp;

        for (const auto& pose : msg->poses)
        {
            geometry_msgs::msg::PoseStamped pose_stamped, transformed_pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.pose = pose;

            try
            {
                tf_buffer_.transform(pose_stamped, transformed_pose_stamped, "map", tf2::durationFromSec(1.0));
                transformed_poses.poses.push_back(transformed_pose_stamped.pose);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            }
        }

        cone_pose_map_pub_->publish(transformed_poses);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_pose_map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConePoseTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
