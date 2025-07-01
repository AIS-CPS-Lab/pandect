#include <event_camera_codecs/event_processor.h>
#include <math.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <iostream>
#include <stdexcept>
#include <vector>

class PointCloudProcessor : public event_camera_codecs::EventProcessor {
   public:
    PointCloudProcessor(unsigned int particle_count)
        : particle_count_(particle_count) {
        point_cloud_template_ = sensor_msgs::msg::PointCloud2();
        point_cloud_template_.height = 1;  // PointCloud2 is typically flat
        point_cloud_template_.width = particle_count_;
        point_cloud_template_.is_dense = true;       // No NaNs in the data
        point_cloud_template_.is_bigendian = false;  // Little-endian by default
        point_cloud_template_.fields.resize(4);      // x, y, time fields
        // Define the fields for the PointCloud2 message
        // Define the x field as INT16
        point_cloud_template_.fields[0].name = "x";
        point_cloud_template_.fields[0].offset = 0;
        point_cloud_template_.fields[0].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_template_.fields[0].count = 1;
        // Define the y field as INT16
        point_cloud_template_.fields[1].name = "y";
        point_cloud_template_.fields[1].offset = sizeof(float);
        point_cloud_template_.fields[1].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_template_.fields[1].count = 1;
        // Define the time field as UINT64
        point_cloud_template_.fields[2].name = "z";
        point_cloud_template_.fields[2].offset = sizeof(float) * 2;
        point_cloud_template_.fields[2].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_template_.fields[2].count = 1;
        // Define the polarity field as UINT8
        point_cloud_template_.fields[3].name = "polarity";
        point_cloud_template_.fields[3].offset = sizeof(float) * 3;
        point_cloud_template_.fields[3].datatype =
            sensor_msgs::msg::PointField::UINT8;
        point_cloud_template_.fields[3].count = 1;

        // Set the point step and row step
        point_cloud_template_.point_step =
            sizeof(float) * 3 + sizeof(uint8_t);  // 3 fields of float32
        point_cloud_template_.row_step =
            point_cloud_template_.point_step * point_cloud_template_.width;
        point_cloud_template_.data.resize(point_cloud_template_.row_step *
                                          point_cloud_template_.height);
    }
    ~PointCloudProcessor() override = default;

    inline void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey,
                        uint8_t polarity) override {
        // Add the event data to the point cloud
        // Updates a new particle in the point cloud with the event data
        int index = current_particle_index_ * point_cloud_template_.point_step;
        float x = static_cast<float>(ex) / 100.0f;  // Convert to meters
        float y = static_cast<float>(ey) / 100.0f;
        float z = static_cast<float>(sensor_time) / 1e9f;
        z = std::fmod(z, 1.0f);

        uint8_t polarity_value = static_cast<uint8_t>(polarity);
        std::memcpy(&point_cloud_template_.data[index], &x, sizeof(float));
        std::memcpy(&point_cloud_template_.data[index + sizeof(float)], &y,
                    sizeof(float));
        std::memcpy(&point_cloud_template_.data[index + sizeof(float) * 2], &z,
                    sizeof(float));
        std::memcpy(&point_cloud_template_.data[index + sizeof(float) * 3],
                    &polarity_value, sizeof(uint8_t));

        // Updates the index for the next particle
        current_particle_index_++;
        current_particle_index_ %= particle_count_;
    }

    void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}

    void finished() override {}

    void rawData(const char*, size_t) override{};  // passthrough of raw data

    sensor_msgs::msg::PointCloud2 getPointCloud() const {
        // Returns the current point cloud data
        return point_cloud_template_;
    }

   private:
    unsigned int particle_count_;
    unsigned int current_particle_index_ = 0;
    sensor_msgs::msg::PointCloud2 point_cloud_template_;
    rclcpp::Logger logger_ = rclcpp::get_logger("PointCloudProcessor");
    rclcpp::Clock clock_;
};
