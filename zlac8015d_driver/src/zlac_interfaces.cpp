#include "zlac8015d_driver/zlac_interfaces.hpp"
#include "zlac8015d_driver/zlac8015d_driver.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

Zlac_Interfaces::Zlac_Interfaces() : Node("zlac_driver_node")
{
    this->declare_parameter<std::string>("modbus_port", "/dev/ttyUSB0");

    std::string modbus_port = this->get_parameter("modbus_port").as_string();
    bldcMotor = std::make_shared<ZLAC8015D_API>(modbus_port);

    if (bldcMotor->isConnected())
    {
        bldcMotor->disableMotor();
        bldcMotor->setAccelTime(1000, 1000);
        bldcMotor->setDecelTime(1000, 1000);
        bldcMotor->enableMotor();
        bldcMotor->setRPM(0, 0);
        RCLCPP_INFO(this->get_logger(), "✅ Động cơ đã khởi tạo thành công!");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Không thể kết nối với động cơ!");
        bldcMotor.reset();
        return;
    }

    wheel_JointState_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("nhatbot_firmware/JointState", 10);
    wheelVelocities_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "nhatbot_controller/wheel_rotational_vel", 10,
        std::bind(&Zlac_Interfaces::sub_Vel_Callback, this, std::placeholders::_1));

    motor_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "nhatbot_firmware/reset_feedback_position",
        std::bind(&Zlac_Interfaces::ResetPos_Callback, this, std::placeholders::_1, std::placeholders::_2));

    timer_JointState_ = this->create_wall_timer(
        100ms, std::bind(&Zlac_Interfaces::pub_JointState_Callback, this));
}

Zlac_Interfaces::~Zlac_Interfaces()
{
    exitBLDCMotor();
}

// void Zlac_Interfaces::pub_JointState_Callback()
// {
//     if (!bldcMotor || !bldcMotor->isConnected())
//     {
//         return;
//     }

//     try
//     {
//         sensor_msgs::msg::JointState msg;
//         // Thay vì tạo std::pair từ các giá trị, bạn có thể gọi trực tiếp
//         const auto &velocities = bldcMotor->getAngularVelocity(); // Tham chiếu const để tránh copy không cần thiết

//         double left_vel = velocities.first;
//         double right_vel = velocities.second;

//         auto [left_pos, right_pos] = bldcMotor->getWheelsTravelled();

//         msg.velocity = {left_vel, right_vel};
//         msg.position = {left_pos, right_pos};
//         msg.header.stamp = this->now();

//         wheel_JointState_pub_->publish(msg);
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "❌ Lỗi khi xuất JointState: %s", e.what());
//     }
// }

void Zlac_Interfaces::pub_JointState_Callback()
{
    if (!bldcMotor || !bldcMotor->isConnected())
    {
        return;
    }

    try
    {
        sensor_msgs::msg::JointState msg;

        // Gọi getAngularVelocity() để lấy vận tốc góc của bánh trái và phải
        std::pair<double, double> velocities = bldcMotor->getAngularVelocity();

        double left_vel = velocities.first;
        double right_vel = velocities.second;

        // Lấy vị trí bánh xe
        auto [left_pos, right_pos] = bldcMotor->getWheelsTravelled();

        msg.velocity = {left_vel, right_vel};
        msg.position = {left_pos, right_pos};
        msg.header.stamp = this->now();

        wheel_JointState_pub_->publish(msg);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Lỗi khi xuất JointState: %s", e.what());
    }
}

void Zlac_Interfaces::sub_Vel_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!bldcMotor || !bldcMotor->isConnected())
    {
        RCLCPP_WARN(this->get_logger(), "⚠ Mất kết nối Modbus! Đang thử kết nối lại...");
        bldcMotor->reconnect();

        if (!bldcMotor->isConnected())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Thử kết nối lại Modbus thất bại! Không gửi lệnh.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ Kết nối lại Modbus thành công! Tiếp tục điều khiển động cơ.");
    }

    if (msg->data.size() < 2)
    {
        return;
    }

    double leftWheel = msg->data[0] * (60.0 / (2 * M_PI));
    double rightWheel = msg->data[1] * (60.0 / (2 * M_PI));

    double battery_voltage = bldcMotor->getBatteryVoltage();
    RCLCPP_INFO(this->get_logger(), "🔋 Điện áp pin: %.2fV", battery_voltage);

    leftWheel = std::abs(leftWheel) < 0.5 ? 0.0 : leftWheel;
    rightWheel = std::abs(rightWheel) < 0.5 ? 0.0 : rightWheel;

    try
    {
        bldcMotor->setRPM(static_cast<int>(-leftWheel), static_cast<int>(rightWheel));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Lỗi khi điều khiển động cơ: %s", e.what());
    }
}

void Zlac_Interfaces::ResetPos_Callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    (void)request; // Tránh cảnh báo unused parameter
    response->success = true;
    response->message = "Encoder position reset!";
}

void Zlac_Interfaces::exitBLDCMotor()
{
    if (bldcMotor)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "🛑 Dừng động cơ ngay lập tức!");
            bldcMotor->setRPM(0, 0);
            bldcMotor->close_connect();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Không thể dừng động cơ: %s", e.what());
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zlac_Interfaces>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Lỗi trong main(): %s", e.what());
    }

    // 🚀 Đảm bảo động cơ được dừng khi tắt ROS node
    if (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "✅ Zlac_Interfaces Node đã dừng hoàn toàn!");
    }

    node->exitBLDCMotor();    // 🛑 Dừng động cơ trước khi thoát
    node->~Zlac_Interfaces(); // Giải phóng bộ nhớ

    rclcpp::shutdown();
    return 0;
}
