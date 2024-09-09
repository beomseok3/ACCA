#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>
#include <vector>
#include <cstring>

using std::placeholders::_1;

class VelodyneSubscriber : public rclcpp::Node
{
public:
    VelodyneSubscriber(const rclcpp::NodeOptions& options) : Node("bs_cropbox_filter", options)
    {
        this->declare_parameter("mode", 0);  // Declare a parameter with a default value
        this->get_parameter("mode", mode_);  // Retrieve the parameter value

        RCLCPP_INFO(this->get_logger(), "Mode: %d", mode_);

        velodyne_raw_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&VelodyneSubscriber::velodyneCallback, this, _1));

        cropbox_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropbox_filtered", 10);
    }

private:
    void velodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<uint8_t> filtered_data;

        int x_offset = -1;
        int y_offset = -1;
        for (const auto &field : msg->fields) {
            if (field.name == "x") {
                x_offset = field.offset;
            } else if (field.name == "y") {
                y_offset = field.offset;
            }
        }
        if (x_offset == -1 || y_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "PointCloud2 message does not have 'x' or 'y' fields.");
            return;
        }

        const size_t float_size = 4;
        float min_x, max_x, min_y, max_y;
        if (mode_ == 0) {
            min_x = 0.0;
            max_x = 3.0;
            min_y = -10.0;
            max_y = 0.0;
        } else if (mode_ == 1) {
            min_x = 0.0;
            max_x = 10.0;  
            min_y = -2.5;
            max_y = 2.5;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported mode: %d. Defaulting to mode 0.", mode_);
            min_x = 0.0;
            max_x = 3.0;
            min_y = -25.0;
            max_y = 0.0;
        }
        std::cout << "x: " << min_x << " ~ " << max_x << "    " << "y: " << min_y << " ~ " << max_y << std::endl;

        for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
        {
            float x, y;
            std::memcpy(&x, &msg->data[i + x_offset], float_size);
            std::memcpy(&y, &msg->data[i + y_offset], float_size);
            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y)
            {
                for (size_t j = 0; j < msg->point_step; ++j)
                {
                    filtered_data.push_back(msg->data[i + j]);
                }
            }
        }

        auto processed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        processed_msg->header = msg->header;
        processed_msg->height = 1;
        processed_msg->width = filtered_data.size() / msg->point_step;
        processed_msg->fields = msg->fields;
        processed_msg->is_bigendian = msg->is_bigendian;
        processed_msg->point_step = msg->point_step;
        processed_msg->row_step = processed_msg->point_step * processed_msg->width;
        processed_msg->data = filtered_data;
        processed_msg->is_dense = true;

        if (processed_msg->width * processed_msg->point_step != processed_msg->data.size()) {
            RCLCPP_ERROR(this->get_logger(), "Data size (%zu bytes) does not match width (%u) times point_step (%u).",
                         processed_msg->data.size(), processed_msg->width, processed_msg->point_step);
        } else {
            cropbox_publisher_->publish(*processed_msg);
        }
    }

    int mode_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropbox_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_raw_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    if (argc > 1) {
        options.append_parameter_override("mode", std::atoi(argv[1]));  // Set the mode parameter from the command-line argument
    }

    auto node = std::make_shared<VelodyneSubscriber>(options);  

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
