#pragma once

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pandect_event_example/example_processor.hpp"

namespace pandect_event_example {
/**
 * @brief PandectEventExampleNode class is a ROS 2 node that processes events
 */
class PandectEventExampleNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for the PandectEventExampleNode class.
     *
     * @param options node options for configuring the node
     */
    explicit PandectEventExampleNode(const rclcpp::NodeOptions& options);

   private:
    /**
     * @brief Sets up subscribers, publishers, etc. to configure the node
     */
    void setup();

    // Defining a short alias for the event packet type
    // This is useful for readability and to avoid long type names in the code
    using EventPacket = event_camera_msgs::msg::EventPacket;
    /**
     * @brief Callback function for processing incoming event packets.
     * @param msg The event packet message containing events to be processed.
     * This function decodes the event packet and processes each event using the
     * ExampleProcessor.
     */
    void process_event_packet(const EventPacket::SharedPtr msg);

    /**
     * @brief Callback function for processing incoming event packets.
     * @param msg The event packet message containing events to be processed.
     * This function decodes the event packet and processes each event using the
     * ExampleProcessor.
     * @note Note the SharedPtr type for the message, which is a common pattern
     * in ROS 2 with components. This allows for zero-copy message passing.
     */
    rclcpp::Subscription<EventPacket>::SharedPtr event_packet_subscriber_;

    /**
     * The decoder factory is used for creating a decoder instance
     */
    event_camera_codecs::DecoderFactory<EventPacket, ExampleProcessor>
        decoder_factory_;

    /**
     * The decoder instance is used to decode event packets
     * It takes in the event packet messages and decodes them into events
     * that can be processed by the ExampleProcessor.
     * @note For some reason, the decoder must be a pointer.
     */
    event_camera_codecs::Decoder<EventPacket, ExampleProcessor>* decoder_;

    /**
     * The example processor is used to process the events received from the
     * event packet It implements the event processing logic, such as logging
     * and counting events.
     */
    std::shared_ptr<ExampleProcessor> processor_;
};

}  // namespace pandect_event_example
