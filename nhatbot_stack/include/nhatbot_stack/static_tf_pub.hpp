#ifndef STATIC_TF_PUB_HPP_
#define STATIC_TF_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
    StaticFramePublisher();

private:
    void publish_static_tf();

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

#endif  // STATIC_TF_PUB_HPP_
