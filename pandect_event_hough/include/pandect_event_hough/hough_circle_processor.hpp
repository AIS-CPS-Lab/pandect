#include <event_camera_codecs/event_processor.h>
#include <math.h>
#include <vector>
#include <stdexcept>
#include <iostream>

class HoughCircleProcessor : public event_camera_codecs::EventProcessor
{
public:
    HoughCircleProcessor(
        unsigned int image_width, // image width
        unsigned int image_height, // image height
        unsigned int num_steps_angle, // number of steps in theta
        unsigned int num_steps_radius, // number of steps in radius
        float min_radius, // minimum radius to consider
        float max_radius) // maximum radius to consider
        : accumulator(
            image_width * image_height, 0), 
            image_width(image_width), 
            image_height(image_height), 
            num_steps_angle(num_steps_angle),
            num_steps_radius(num_steps_radius), 
            min_radius(min_radius), 
            max_radius(max_radius) 
        {
            if (image_width == 0 
                || image_height == 0 
                || num_steps_angle == 0  
                || num_steps_radius == 0
                || min_radius >= max_radius) {
                std::cerr << "Parameters tried to initialized in HoughCircleProcessor: "
                          << "width=" << image_width << ", height=" << image_height
                          << ", num_steps_angle=" << num_steps_angle
                          << ", num_steps_radius=" << num_steps_radius
                          << ", min_radius=" << min_radius
                          << ", max_radius=" << max_radius << std::endl;
                throw std::invalid_argument("Invalid parameters for HoughCircleProcessor");
            }
            radius_increment = (max_radius - min_radius) / num_steps_radius; // Calculate the radius increment based on the number of steps
            // radius_increment = (max_radius - min_radius) / static_cast<float>(num_steps_radius);
            unit_circle_points = std::vector<std::pair<float, float>>();
            for (unsigned int i = 0; i < num_steps_angle; ++i) {
                float theta = 2.0f * M_PI * i / num_steps_angle;
                unit_circle_points.emplace_back(std::cos(theta), std::sin(theta));
            }
            accumulator.resize(
                (max_radius - min_radius) / radius_increment * image_width * image_height, 0);
        }
    ~HoughCircleProcessor() override = default;


    inline void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override {
        // Convert event coordinates to accumulator index
        int x = static_cast<int>(ex);
        int y = static_cast<int>(ey);

        // Iterate over all possible radii and angles
        for (float r = min_radius; r <= max_radius; r += radius_increment) {
            for (const auto& [dx, dy] : unit_circle_points) {
                int x0 = static_cast<int>(x - r * dx);
                int y0 = static_cast<int>(y - r * dy);

                // Check if the point is within bounds
                if (x0 >= 0 && x0 < image_width 
                    && y0 >= 0 && y0 < image_height) {
                    accumulator[getAccumulatorIndex(r, x0, y0)] += 1; // Increment or decrement based on polarity
                }
            }
        }
    }

    void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}

    void finished() override {
        // Process the accumulator to find circles
        std::vector<Circle> detected_circles;
        for (float r = min_radius; r <= max_radius; r += radius_increment) {
            for (unsigned int y = 0; y < image_height; ++y) {
                for (unsigned int x = 0; x < image_width; ++x) {
                    int index = getAccumulatorIndex(r, x, y);
                    if (index < 0 || index >= static_cast<int>(accumulator.size())) {
                        continue; // Skip out of bounds
                    }
                    if (accumulator[index] > 0) { // Threshold can be adjusted
                        detected_circles.push_back(Circle{x, y, r, accumulator[index]});
                    }
                }
            }
        }

        // Sort detected circles by votes (descending)
        std::sort(detected_circles.begin(), detected_circles.end(),
                  [](const Circle& a, const Circle& b) {
                      return a.votes > b.votes;
                  });
        int top_n = std::min(10, static_cast<int>(circles.size()));
        best_circles.assign(circles.begin(), circles.begin() + top_n);

        // Reset the accumulator for the next packet
        std::fill(accumulator.begin(), accumulator.end(), 0);

        // Output detected circles
        if (!detected_circles.empty()) {
            const Circle* best_circle = &detected_circles[0];
            for (const auto& circle : detected_circles) {
                if (circle.votes > best_circle->votes) {
                    best_circle = &circle;
                }
            }
            std::cout << "Best circle: " << best_circle->toString() << std::endl;
            this->best_circle = *best_circle; // Store the best circle found
        } else {
            std::cout << "No circles detected." << std::endl;
        }
    }; // called after no more events decoded in this packet


    void rawData(const char *, size_t) override{};  // passthrough of raw data

    struct Circle {
        unsigned int x; // x-coordinate of the circle center
        unsigned int y; // y-coordinate of the circle center
        float radius; // radius of the circle
        int votes; // number of votes for this circle

        std::string toString() const {
            return "Circle(x: " + std::to_string(x) + 
                    ", y: " + std::to_string(y) + 
                    ", radius: " + std::to_string(radius) + 
                    ", votes: " + std::to_string(votes) + ")";
        }
    };

    std::vector<Circle> getBestCircles() const {
        return best_circles; // Return the best circle found in the current packet
    }
    
private:
    std::vector<int> accumulator;
    unsigned int image_width;
    unsigned int image_height;
    unsigned int num_steps_angle;
    unsigned int num_steps_radius;
    float radius_increment;
    float min_radius;
    float max_radius;
    std::vector<std::pair<float, float>> unit_circle_points; // precomputed unit circle points for radius calculations

    int getRadiusIndex(float radius) const {
        if (radius < min_radius || radius > max_radius) {
            throw std::out_of_range("Radius out of bounds");
        }
        return static_cast<int>((radius - min_radius) * num_steps_radius); // Calculate index based on radius
    }

    int getAccumulatorIndex(float radius, unsigned int x, unsigned int y) const {
        if (x >= image_width || y >= image_height) {
            throw std::out_of_range("Coordinates out of bounds");
        }
        return getRadiusIndex(radius) + y * image_width + x; // Calculate index based on radius and coordinates
    }

    std::vector<Circle> best_circles; // best circle found in the current packet
};