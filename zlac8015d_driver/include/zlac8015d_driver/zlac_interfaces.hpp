#ifndef ZLAC_INTERFACES_HPP
#define ZLAC_INTERFACES_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "zlac8015d_driver/zlac8015d_driver.hpp"

enum class NhatBot_Params {
    SET_PORT = 0,
    SET_JOINT_STATE_TIMER = 100,
    SET_ACCEL_TIME = 1000,
    SET_DECEL_TIME = 1000,
    SET_VELOCITY_MODE = 3,
    SET_STOP_RPM = 0,
    SET_CLEAR_FB_POS = 3,
    
};
static constexpr double SMALL_VELOCITY_THRESHOLD = 0.5;

class Zlac_Interfaces : public rclcpp::Node {
private:
    std::shared_ptr<ZLAC8015D_API> bldcMotor;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_JointState_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheelVelocities_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motor_srv_;
    rclcpp::TimerBase::SharedPtr timer_JointState_;

    
    void pub_JointState_Callback();
    void sub_Vel_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void ResetPos_Callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                           std_srvs::srv::Trigger::Response::SharedPtr response);
    

public:
    Zlac_Interfaces();
    void exitBLDCMotor();
    ~Zlac_Interfaces();
};

#endif // ZLAC_INTERFACES_HPP
