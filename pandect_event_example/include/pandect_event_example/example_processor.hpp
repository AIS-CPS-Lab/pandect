#pragma once

#include <event_camera_codecs/event_processor.h>

#include <rclcpp/rclcpp.hpp>

namespace pandect_event_example {
/**
 * @file example_processor.hpp
 * @brief Example implementation of an event processor for event_camera_codecs.
 * This processor logs events and counts them, demonstrating how to handle
 * events. If you want to process events, you can implement your own processor
 * by inheriting from the event_camera_codecs::EventProcessor class. It has four
 * virtual functions that you need to implement:
 * - eventCD: Called for every event received.
 * - eventExtTrigger: Called for every external trigger event received.
 * - finished: Called when the processing of the event stream is finished.
 * - rawData: Called before the other functions at the start of every event
 * stream Both eventExtTrigger and rawData are optional for most cases.
 */
class ExampleProcessor : public event_camera_codecs::EventProcessor {
   public:
    /**
     * @brief Constructor for the ExampleProcessor class.
     * Initializes the processor
     */
    ExampleProcessor() : event_count_(0) {
        RCLCPP_DEBUG(logger_, "ExampleProcessor initialized");
    }

    /**
     * @brief Destructor for the ExampleProcessor class.
     * Cleans up resources used by the processor
     */
    ~ExampleProcessor() override = default;

    /**
     * This function is called for every event received
     * @param sensor_time The timestamp of the event in nanoseconds
     * @param ex The x coordinate of the event
     * @param ey The y coordinate of the event
     * @param polarity The polarity of the event, 0 for negative and 1 for
     * positive
     * @note The sensor_time is in nanoseconds, but the first 16 seconds (24
     * bits) might be reset
     */
    inline void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey,
                        uint8_t polarity) override {
        // example processing logic
        // Convert nanoseconds to seconds
        float time = static_cast<float>(sensor_time) / 1e9;
        // Log a debug event, but only every 1000 ms
        RCLCPP_DEBUG_THROTTLE(
            logger_, clock_, 1000,
            "Event received: time=%2f, x=%u, y=%u, polarity=%u", time, ex, ey,
            polarity);
        // Increment the event count
        // This is just an example, you can process the event as needed
        event_count_++;
    }

    /**
     * This function is called for every external trigger event received
     * @param sensor_time The timestamp of the event in nanoseconds
     * @param edge The edge of the trigger, 0 for falling and 1 for rising
     * @param id The ID of the trigger, can be used to differentiate between
     * multiple triggers
     * @details The camera allows you to insert a trigger label into the data
     * stream This can be used to synchronize with other sensors or events. This
     * function will be called for every external trigger event received.
     */
    void eventExtTrigger(uint64_t sensor_time, uint8_t edge,
                         uint8_t id) override {}

    /**
     * This function is called when the processing of the event stream is
     * finished it happens when all the current events have been processed and
     * no new events are expected.
     */
    void finished() override {
        // Log the total number of events processed
        RCLCPP_INFO(logger_, "Finished processing events. Total events: %u",
                    event_count_);
        // Reset the event count for the next stream
        event_count_ = 0;
    }

    /**
     * This function is called before every stream of events
     * It contains all the raw data received from the camera
     * @param data The raw data received from the camera
     * @param len The length of the raw data in bytes
     */
    void rawData(const char* data, size_t len) override{};

   private:
    unsigned int event_count_{0};  // Counter for the number of events processed
    rclcpp::Logger logger_ = rclcpp::get_logger("ExampleProcessor");
    rclcpp::Clock clock_;
};
}  // namespace pandect_event_example
