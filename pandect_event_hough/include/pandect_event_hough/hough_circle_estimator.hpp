#pragma once

#include <vector>
#include "hough_circle_processor.hpp"
#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/decoder_factory.h"
#include "event_camera_msgs/msg/event_packet.hpp"

namespace pandect_event_hough {

using EventPacket = event_camera_msgs::msg::EventPacket;
using Decoder = event_camera_codecs::Decoder<EventPacket, HoughCircleProcessor>;

class HoughTransformEstimator {
public:
    HoughTransformEstimator(
        unsigned int num_steps_angle = 360, // default number of steps in theta
        unsigned int num_steps_radius = 100, // default number of steps in radius
        unsigned int min_radius = 5, // default minimum radius to consider
        unsigned int max_radius = 50 // default maximum radius to consider
        ) :
            hough_circle_processor_(nullptr),
            decoder_(nullptr),
            num_steps_angle_(num_steps_angle),
            num_steps_radius_(num_steps_radius),
            min_radius_(min_radius),
            max_radius_(max_radius)
    {
        if (num_steps_angle == 0 || num_steps_radius == 0 || min_radius >= max_radius) {
            throw std::invalid_argument("Invalid parameters for HoughTransformEstimator");
        }
    }

    bool needToInitialize(
        const std::string &encoding, // encoding type
        unsigned int image_width, // image width
        unsigned int image_height // image height
        ) const {
        if (decoder_) {
            return encoding_ != encoding ||
                   decoder_->getWidth() != image_width ||
                   decoder_->getHeight() != image_height ||
                   !hough_circle_processor_;
        } else {
            return true; // Decoder is not initialized, so we need to initialize it
        }
    }

    void initializeCameraParameters(
        std::string encoding, // encoding type
        unsigned int image_width, // image width
        unsigned int image_height // image height
        ) {
        decoder_ = std::unique_ptr<Decoder>(decoder_factory_.getInstance(
            encoding,
            image_width,
            image_height));
        updateProcessorParameters();
    }

    unsigned int getNumStepsAngle() const {
        return num_steps_angle_;
    }
    unsigned int getNumStepsRadius() const {
        return num_steps_radius_;
    }
    unsigned int getMinRadius() const {
        return min_radius_;
    }
    unsigned int getMaxRadius() const {
        return max_radius_;
    }

    HoughCircleProcessor::Circle getBestCircle() const {
        if (!hough_circle_processor_) {
            throw std::runtime_error("HoughCircleProcessor is not initialized.");
        }
        return hough_circle_processor_->getBestCircle();
    }

    void setRadius(
        unsigned int min_radius, // minimum radius to consider
        unsigned int max_radius // maximum radius to consider
    ) {
        if (min_radius >= max_radius) {
            throw std::invalid_argument("Invalid radius range for HoughCircleProcessor");
        }
        min_radius_ = min_radius;
        max_radius_ = max_radius;   
        updateProcessorParameters();
    }

    void setRadiusSteps(
        unsigned int num_steps_radius // number of steps in radius
    ) {
        if (num_steps_radius == 0) {
            throw std::invalid_argument("Number of steps in radius must be greater than zero");
        }
        num_steps_radius_ = num_steps_radius;
        updateProcessorParameters();
    }

    void setAngleSteps(
        unsigned int num_steps_angle // number of steps in theta
    ) {
        if (num_steps_angle == 0) {
            throw std::invalid_argument("Number of steps in angle must be greater than zero");
        }
        num_steps_angle_ = num_steps_angle;
        updateProcessorParameters();
    }

    void update(const uint8_t *events, size_t numEvents) {
        if (!decoder_) {
            throw std::runtime_error("Decoder is not initialized.");
        }
        if (!hough_circle_processor_) {
            throw std::runtime_error("HoughCircleProcessor is not initialized.");
        }
        decoder_->decode(events, numEvents, hough_circle_processor_.get());
    }

private:
    void updateProcessorParameters() {
        if (decoder_) {
            hough_circle_processor_ = std::make_unique<HoughCircleProcessor>(
                decoder_->getWidth(),
                decoder_->getHeight(),
                num_steps_angle_,
                num_steps_radius_,
                min_radius_,
                max_radius_);
        }  
    }

    std::unique_ptr<HoughCircleProcessor> hough_circle_processor_;
    std::unique_ptr<Decoder> decoder_;
    event_camera_codecs::DecoderFactory<EventPacket, HoughCircleProcessor> decoder_factory_;
    std::string encoding_;
    unsigned int num_steps_angle_;
    unsigned int num_steps_radius_;
    unsigned int min_radius_;
    unsigned int max_radius_;
};

} // namespace pandect_event_hough
