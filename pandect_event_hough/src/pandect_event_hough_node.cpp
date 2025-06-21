#include <functional>

// Update the include path below if the header is in the same directory or
// adjust as needed
#include <rclcpp_components/register_node_macro.hpp>

#include "pandect_event_hough/pandect_event_hough_node.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pandect_event_hough::HoughTransformNode)

namespace pandect_event_hough {

HoughTransformNode::HoughTransformNode(const rclcpp::NodeOptions& options)
    : Node("pandect_event_hough_node", options) {
    std::cout << "HoughTransformNode constructor called" << std::endl;
    hough_transform_estimator_ = HoughTransformEstimator();
    RCLCPP_INFO(this->get_logger(),
                "HoughTransformNode initialized with default parameters");

    // subscriber for handling incoming messages
    subscriber_ =
        this->create_subscription<event_camera_msgs::msg::EventPacket>(
            "~/events", rclcpp::SensorDataQoS(),
            std::bind(&HoughTransformNode::eventMsgCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
                subscriber_->get_topic_name());

    // publisher for publishing outgoing messages
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output", 10);
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
                publisher_->get_topic_name());
}

void HoughTransformNode::eventMsgCallback(
    const event_camera_msgs::msg::EventPacket::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Message received: '%d'", 2);

    if (hough_transform_estimator_.needToInitialize(msg->encoding, msg->width,
                                                    msg->height)) {
        hough_transform_estimator_.initializeCameraParameters(
            msg->encoding, msg->width, msg->height);
        RCLCPP_INFO(this->get_logger(),
                    "HoughTransformEstimator initialized with encoding: '%s', "
                    "width: %u, height: %u",
                    msg->encoding.c_str(), msg->width, msg->height);
    }
    RCLCPP_INFO(this->get_logger(), "Processing events from message");
    hough_transform_estimator_.update(msg->events.data(), msg->events.size());

    RCLCPP_INFO(this->get_logger(), "Events processed: %zu",
                msg->events.size());
    if (hough_transform_estimator_.getBestCircle().votes > 0) {
        RCLCPP_INFO(
            this->get_logger(), "Best circle found: %s",
            hough_transform_estimator_.getBestCircle().toString().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "No circles detected in this packet.");
    }
    RCLCPP_INFO(this->get_logger(), "Processing complete for this packet.");
}

void publishTransformImage() {
    int width = 1280, height = 720;

    // Create blank image (grayscale or RGB)
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    // Draw circles
    for (const auto& c : getBestCircles()) {
        cv::circle(image, cv::Point(c.x, c.y), c.radius, cv::Scalar(0, 255, 0),
                   2);  // Green circle
    }

    // Convert to ROS2 message
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    image_pub_->publish(*msg);
}

}  // namespace pandect_event_hough
