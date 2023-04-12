#ifndef TRACKING__OBJECTDETECTOR_HPP_
#define TRACKING__OBJECTDETECTOR_HPP_

#include <memory>
#include <vector>

#include "vision_msgs/msg/detection2_d.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tracking
{
class ObjectDetector : public rclcpp::Node
{


public:
    ObjectDetector();

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;

    std::vector<double> hsv_filter_ranges_ {0, 180, 0, 255, 0, 255};
    bool debug_ {true};
};
}

#endif