#include <functional>
#include <pandect_event_example/pandect_event_example_node.hpp>

// This is a ROS 2 component, so we need to register it with the component
// manager You can use the RCLCPP_COMPONENTS_REGISTER_NODE macro to do this.
// where your node class is the argument to the macro.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pandect_event_example::PandectEventExampleNode)

namespace pandect_event_example {

PandectEventExampleNode::PandectEventExampleNode(
    const rclcpp::NodeOptions& options)
    : Node("pandect_event_example_node", options) {
    this->processor_ = std::make_shared<ExampleProcessor>();
    this->setup();
}

void PandectEventExampleNode::setup() {
    event_packet_subscriber_ = this->create_subscription<EventPacket>(
        "~/events",               // Remap using launch file or command line
        rclcpp::SensorDataQoS(),  // Use SensorDataQoS for event packets
        std::bind(&PandectEventExampleNode::process_event_packet, this,
                  std::placeholders::_1));
}

void PandectEventExampleNode::process_event_packet(
    const EventPacket::SharedPtr msg) {
    // Ensure the decoder is created only once
    // This is initializing the decoder
    // You could also do this during setup, but then you
    // would neet to know the encoding, width, and height beforehand.
    if (!decoder_) {
        decoder_ = decoder_factory_.getInstance(msg->encoding, msg->width,
                                                msg->height);
    }
    // Decode the event packet using the decoder
    // This will call the ExampleProcessor for each event in the packet
    decoder_->decode(&(msg->events[0]), msg->events.size(),
                     this->processor_.get());
}

}  // namespace pandect_event_example
