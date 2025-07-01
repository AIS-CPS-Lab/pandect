#pragma once

#include <vector>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/decoder_factory.h"
#include "event_camera_msgs/msg/event_packet.hpp"
#include "hough_circle_processor.hpp"

namespace pandect_event_hough {

using EventPacket = event_camera_msgs::msg::EventPacket;
using Decoder = event_camera_codecs::Decoder<EventPacket, HoughCircleProcessor>;

class HoughTransformEstimator {
   public:
    HoughTransformEstimator(
        unsigned int num_steps_angle = 36,  // default number of steps in theta
        unsigned int num_steps_radius =
            20,                        // default number of steps in radius
        unsigned int min_radius = 50,  // default minimum radius to consider
        unsigned int max_radius = 800  // default maximum radius to consider
        )
        : hough_circle_processor_(nullptr),
          decoder_(nullptr),
          num_steps_angle_(num_steps_angle),
          num_steps_radius_(num_steps_radius),
          min_radius_(min_radius),
          max_radius_(max_radius),
          image_width_(0),
          image_height_(0) {
        if (num_steps_angle == 0 || num_steps_radius == 0 ||
            min_radius >= max_radius) {
            throw std::invalid_argument(
                "Invalid parameters for HoughTransformEstimator");
        }
    }

    bool needToInitialize(const std::string &encoding,  // encoding type
                          unsigned int image_width,     // image width
                          unsigned int image_height     // image height
    ) {
        if (decoder_) {
            bool needUpdate =
                encoding_ != encoding || image_width_ != image_width ||
                image_height_ != image_height || !hough_circle_processor_;
            if (needUpdate) {
                encoding_ = encoding;
                image_width_ = image_width;
                image_height_ = image_height;
                return true;  // Decoder needs to be initialized with new
                              // parameters
            }
            return false;  // Decoder is already initialized with the same
                           // parameters
        } else {
            return true;  // Decoder is not initialized, so we need to
                          // initialize it
        }
    }

    void initializeCameraParameters(std::string encoding,      // encoding type
                                    unsigned int image_width,  // image width
                                    unsigned int image_height  // image height
    ) {
        decoder_ =
            decoder_factory_.getInstance(encoding, image_width, image_height);
        updateProcessorParameters();
    }

    unsigned int getNumStepsAngle() const { return num_steps_angle_; }
    unsigned int getNumStepsRadius() const { return num_steps_radius_; }
    unsigned int getMinRadius() const { return min_radius_; }
    unsigned int getMaxRadius() const { return max_radius_; }

    std::vector<HoughCircleProcessor::Circle> getBestCircles() const {
        ;
        if (!hough_circle_processor_) {
            throw std::runtime_error(
                "HoughCircleProcessor is not initialized.");
        }
        return hough_circle_processor_->getBestCircles();
    }

    void setRadius(unsigned int min_radius,  // minimum radius to consider
                   unsigned int max_radius   // maximum radius to consider
    ) {
        if (min_radius >= max_radius) {
            throw std::invalid_argument(
                "Invalid radius range for HoughCircleProcessor");
        }
        min_radius_ = min_radius;
        max_radius_ = max_radius;
        updateProcessorParameters();
    }

    void setRadiusSteps(
        unsigned int num_steps_radius  // number of steps in radius
    ) {
        if (num_steps_radius == 0) {
            throw std::invalid_argument(
                "Number of steps in radius must be greater than zero");
        }
        num_steps_radius_ = num_steps_radius;
        updateProcessorParameters();
    }

    void setAngleSteps(unsigned int num_steps_angle  // number of steps in theta
    ) {
        if (num_steps_angle == 0) {
            throw std::invalid_argument(
                "Number of steps in angle must be greater than zero");
        }
        num_steps_angle_ = num_steps_angle;
        updateProcessorParameters();
    }

    void update(const uint8_t *events, size_t numEvents) {
        if (!decoder_) {
            throw std::runtime_error("Decoder is not initialized.");
        }
        if (!hough_circle_processor_) {
            throw std::runtime_error(
                "HoughCircleProcessor is not initialized.");
        }
        decoder_->decode(events, numEvents, hough_circle_processor_.get());
    }

   private:
    void updateProcessorParameters() {
        if (decoder_) {
            hough_circle_processor_ = std::make_unique<HoughCircleProcessor>(
                decoder_->getWidth(), decoder_->getHeight(), num_steps_angle_,
                num_steps_radius_, min_radius_, max_radius_);
        }
    }

    std::unique_ptr<HoughCircleProcessor> hough_circle_processor_;
    Decoder *decoder_;
    event_camera_codecs::DecoderFactory<EventPacket, HoughCircleProcessor>
        decoder_factory_;
    std::string encoding_;
    unsigned int image_width_ = 0;
    unsigned int image_height_ = 0;
    unsigned int num_steps_angle_ = 1;
    unsigned int num_steps_radius_ = 1;
    unsigned int min_radius_ = 0;
    unsigned int max_radius_ = 1;
};

}  // namespace pandect_event_hough
