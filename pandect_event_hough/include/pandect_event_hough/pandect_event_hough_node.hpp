#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pandect_event_hough/hough_circle_estimator.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace pandect_event_hough {

/**
 * @brief HoughTransformNode class
 */
class HoughTransformNode : public rclcpp::Node {

 public:

  /**
   * @brief Constructor
   *
   * @param options node options
   */
  explicit HoughTransformNode(const rclcpp::NodeOptions& options);

 private:

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void eventMsgCallback(const event_camera_msgs::msg::EventPacket::ConstSharedPtr msg);

 private:
  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  HoughTransformEstimator hough_transform_estimator_;
};
}
