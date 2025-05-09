#include "nhatbot_stack/static_tf_pub.hpp"

StaticFramePublisher::StaticFramePublisher() : Node("lidar_static_tf_pub")
{
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_tf();
}

void StaticFramePublisher::publish_static_tf()
{
    geometry_msgs::msg::TransformStamped static_transform_stamped;

    // ✅ Sử dụng timestamp hợp lệ để tránh lỗi
    // static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.stamp = rclcpp::Time(0);

    static_transform_stamped.header.frame_id = "base_link";
    static_transform_stamped.child_frame_id = "laser";

    static_transform_stamped.transform.translation.x = 0.16;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.076;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transform_stamped.transform.rotation.x = quat.x();
    static_transform_stamped.transform.rotation.y = quat.y();
    static_transform_stamped.transform.rotation.z = quat.z();
    static_transform_stamped.transform.rotation.w = quat.w();

    broadcaster_->sendTransform(static_transform_stamped);

    RCLCPP_INFO(this->get_logger(), "✅ Static transform published: base_footprint → laser");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticFramePublisher>();

    // ✅ Giữ node chạy để transform tồn tại
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
