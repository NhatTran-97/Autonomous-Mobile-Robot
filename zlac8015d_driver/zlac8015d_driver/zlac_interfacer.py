#!/usr/bin/env python3

import sys
import numpy as np
from enum import Enum
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from zlac8015d_driver import ZLAC8015D_API  # Import driver ZLAC8015D
import pymodbus
import os 

class NhatBot_Params(Enum):
    SET_PORT = "/dev/ttyUSB0"
    SET_JOINT_STATE_TIMER = 0.1  # 100ms
    SET_JOINT_STATE_TOPIC_NAME_PUB = "nhatbot_firmware/JointState"
    SET_WHEEL_VEL_TOPIC_NAME_SUB = "nhatbot_controller/wheel_rotational_vel"
    SET_RESET_FB_POS_SERVICE_NAME_SERVER = "nhatbot_firmware/reset_feedback_position"
    SET_ACCEL_TIME = 1000
    SET_DECEL_TIME = 1000
    SET_VELOCITY_MODE = 3
    SET_STOP_RPM = 0
    SET_CLEAR_FB_POS = 3
    SMALL_VELOCITY_THRESHOLD = 0.5  # Nếu tốc độ < 0.5 RPM thì coi như dừng


class Zlac_Interfaces(Node):
    def __init__(self, nhatbot_params: NhatBot_Params):
        super().__init__('zlac_driver_node')

        self.nhatbot_params = nhatbot_params
        self.declare_parameter("modbus_port", self.nhatbot_params.SET_PORT.value)

        self.ReentGroup = ReentrantCallbackGroup()
        self.bldcMotor = None  

        try:
            modbus_port_ = self.get_parameter("modbus_port").get_parameter_value().string_value
            self.init_system(modbus_port_)
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi khi lấy modbus_port: {str(e)}")
            self.bldcMotor = None

        if self.bldcMotor and self.bldcMotor.is_connected():
            self.wheel_JointState_pub_ = self.create_publisher(
                JointState, self.nhatbot_params.SET_JOINT_STATE_TOPIC_NAME_PUB.value, 10
            )
            self.wheelVelocities_sub_ = self.create_subscription(
                Float32MultiArray, self.nhatbot_params.SET_WHEEL_VEL_TOPIC_NAME_SUB.value, 
                self.sub_Vel_Callback, 10, callback_group=self.ReentGroup
            )
            self.motor_srv_ = self.create_service(
                Trigger, self.nhatbot_params.SET_RESET_FB_POS_SERVICE_NAME_SERVER.value, 
                self.ResetPos_Callback
            )
            self.timer_JointState_ = self.create_timer(
                self.nhatbot_params.SET_JOINT_STATE_TIMER.value, self.pub_JointState_Callback, 
                callback_group=self.ReentGroup
            )

    def init_system(self, port):
        """✅ Khởi tạo kết nối với driver động cơ"""
        if not port or port.strip() == "":
            self.get_logger().error("❌ modbus_port không hợp lệ! Dừng khởi tạo động cơ.")
            return

        try:
            self.bldcMotor = ZLAC8015D_API(port)
            if not self.bldcMotor.is_connected():
                raise Exception("Mất kết nối với động cơ")

            self.bldcMotor.disable_motor()
            self.bldcMotor.set_accel_time(self.nhatbot_params.SET_ACCEL_TIME.value, self.nhatbot_params.SET_ACCEL_TIME.value)
            self.bldcMotor.set_decel_time(self.nhatbot_params.SET_DECEL_TIME.value, self.nhatbot_params.SET_DECEL_TIME.value)
            self.bldcMotor.set_mode(self.nhatbot_params.SET_VELOCITY_MODE.value)
            self.bldcMotor.enable_motor()
            self.bldcMotor.set_rpm(self.nhatbot_params.SET_STOP_RPM.value, self.nhatbot_params.SET_STOP_RPM.value)

            self.get_logger().info("✅ Động cơ đã khởi tạo thành công!")

        except Exception as e:
            self.get_logger().error(f"❌ Lỗi khi khởi động động cơ: {str(e)}")
            self.bldcMotor = None  

    def pub_JointState_Callback(self):
        """✅ Xuất dữ liệu JointState khi động cơ có kết nối hợp lệ"""
        if not self.bldcMotor or not self.bldcMotor.is_connected():
            return  

        try:
            msg_wheel_JointState_ = JointState()                   
            msg_wheel_JointState_.velocity = list(self.bldcMotor.get_angular_velocity())  # rad/s
            msg_wheel_JointState_.position = list(self.bldcMotor.get_wheels_travelled())  # rad
            msg_wheel_JointState_.header.stamp = self.get_clock().now().to_msg()
            self.wheel_JointState_pub_.publish(msg_wheel_JointState_)
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi khi xuất JointState: {str(e)}")

    def sub_Vel_Callback(self, msg):
        """✅ Xử lý dữ liệu vận tốc và gửi lệnh đến motor"""
        
        if not self.bldcMotor or not self.bldcMotor.is_connected():
            if self.bldcMotor.was_connected:  # 🔴 Chỉ in log khi lần đầu mất kết nối
                self.get_logger().warn("⚠ Mất kết nối Modbus! Đang thử kết nối lại...")
                self.bldcMotor.was_connected = False  # Đánh dấu là mất kết nối

            # 🔄 Thử kết nối lại
            self.bldcMotor.reset_motor_after_reconnect()

            # 🚀 Debug: Kiểm tra lại kết nối sau khi thử reconnect
            if not self.bldcMotor.is_connected():
                self.get_logger().error("❌ Thử kết nối lại Modbus thất bại! Không gửi lệnh.")
                return  # ❌ Không gửi lệnh nếu chưa kết nối lại được
            
            # 🔥 Nếu kết nối lại thành công, in thông báo
            self.get_logger().info("✅ Kết nối lại Modbus thành công! Tiếp tục điều khiển động cơ.")
            self.bldcMotor.was_connected = True  # Cập nhật trạng thái

        if len(msg.data) < 2:
            return

        leftWheel = msg.data[0] * (60 / (2 * np.pi))
        rightWheel = msg.data[1] * (60 / (2 * np.pi))

        # battery_voltage = self.bldcMotor.get_battery_voltage()
        # if battery_voltage:
        #     print(f"Dung lượng pin hiện tại: {battery_voltage}V")
        # else:
        #     print("Không thể đọc dung lượng pin!")

        leftWheel = 0.0 if abs(leftWheel) < self.nhatbot_params.SMALL_VELOCITY_THRESHOLD.value else leftWheel
        rightWheel = 0.0 if abs(rightWheel) < self.nhatbot_params.SMALL_VELOCITY_THRESHOLD.value else rightWheel

        try:
            self.bldcMotor.set_rpm(int(-leftWheel), int(rightWheel))
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi khi điều khiển động cơ: {str(e)}")



    def exitBLDCMotor(self):
        """✅ Dừng động cơ khi ROS shutdown"""
        if self.bldcMotor:
            try:
                self.get_logger().info("🛑 Dừng động cơ ngay lập tức!")
                self.bldcMotor.set_rpm(0, 0)  
                self.bldcMotor.close_connect()
            except Exception as e:
                self.get_logger().error(f"❌ Không thể dừng động cơ: {str(e)}")


    def ResetPos_Callback(self, request, response):
        """✅ Reset vị trí encoder của động cơ"""
        if not self.bldcMotor or not self.bldcMotor.is_connected():
            response.success = False
            response.message = '❌ Động cơ chưa kết nối hoặc bị lỗi!'
            return response

        try:
            success = self.bldcMotor.clear_position(self.nhatbot_params.SET_CLEAR_FB_POS.value)
            if success:
                response.success = True
                response.message = '✅ Vị trí encoder đã được reset thành công!'
            else:
                response.success = False
                response.message = '❌ Không thể reset encoder, kiểm tra kết nối!'
        except Exception as e:
            response.success = False
            response.message = f'❌ Lỗi khi reset encoder: {str(e)}'

        return response


def main():
    rclpy.init()
    node = Zlac_Interfaces(NhatBot_Params)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("🛑 Ctrl+C detected! Đang dừng Zlac_Interfaces Node...")

    finally:
        if rclpy.ok():
            node.get_logger().info("✅ Zlac_Interfaces Node đã dừng hoàn toàn!")

        node.exitBLDCMotor()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
