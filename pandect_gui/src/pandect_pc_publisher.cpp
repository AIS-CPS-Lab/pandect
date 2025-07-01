#include <chrono>
#include <memory>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/decoder_factory.h"
#include "event_camera_msgs/msg/event_packet.hpp"
#include "pandect_gui/point_cloud_processor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node {
   public:
    PointCloudPublisher() : Node("point_cloud_publisher_node") {
        point_cloud_processor_ = std::make_shared<PointCloudProcessor>(
            50000);  // Example particle count
        subscriber_ =
            this->create_subscription<event_camera_msgs::msg::EventPacket>(
                "events", rclcpp::SensorDataQoS(),
                std::bind(&PointCloudPublisher::eventMsg, this,
                          std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PointCloudPublisher::timer_callback, this));
    }

    void eventMsg(const event_camera_msgs::msg::EventPacket::SharedPtr msg) {
        if (!decoder_) {
            decoder_ = decoder_factory_.getInstance(msg->encoding, msg->width,
                                                    msg->height);
        }
        decoder_->decode(&(msg->events[0]), msg->events.size(),
                         point_cloud_processor_.get());
    }

   private:
    void timer_callback() {
        auto point_cloud_msg = point_cloud_processor_->getPointCloud();
        point_cloud_msg.header.stamp = this->now();
        point_cloud_msg.header.frame_id = "camera_frame";  // Set your frame ID
        publisher_->publish(point_cloud_msg);
        // RCLCPP_INFO(this->get_logger(), "Published PointCloud2 message");
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr
        subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<PointCloudProcessor> point_cloud_processor_;
    event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket,
                                        PointCloudProcessor>
        decoder_factory_;
    event_camera_codecs::Decoder<event_camera_msgs::msg::EventPacket,
                                 PointCloudProcessor>* decoder_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}
