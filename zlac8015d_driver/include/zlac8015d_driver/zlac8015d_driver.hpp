#ifndef ZLAC8015D_DRIVER_HPP
#define ZLAC8015D_DRIVER_HPP

#include <modbus/modbus.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <array>

class ZLAC8015D_API
{
private:
    modbus_t *ctx;
    int slave_id;
    bool was_connected;
    int reconnect_interval; // thời gian chờ để thử kết nối lại (s)
    double last_attempt_time;

    bool checkConnection();
    std::vector<uint16_t> readRegisters(int addr, int num) const;
    bool writeRegister(int addr, uint16_t value);
    bool writeRegisters(int addr, const std::vector<uint16_t> &values);

public:
    ZLAC8015D_API(const std::string &port, int baudrate = 115200, int slave_id = 1);
    ~ZLAC8015D_API();

    bool isConnected();
    bool reconnect();
    // void exitBLDCMotor();

    // Điều khiển động cơ
    bool enableMotor();
    bool disableMotor();
    bool emergencyStop();
    bool setRPM(int left_rpm, int right_rpm);
    std::pair<int, int> getRPM() const;

    // Kiểm tra trạng thái động cơ
    bool isMotorEnabled();
    std::pair<uint16_t, uint16_t> getMotorFaults();

    std::pair<double, double> getWheelsTravelled();
  

        std::pair<int32_t, int32_t> getWheelsTick();
    int getMotorTemperature();
    int getDriverTemperature();
    double getBatteryVoltage();
    void setAccelTime(int left_ms, int right_ms);
    void setDecelTime(int left_ms, int right_ms);

    void clear_position(int pos);
    void close_connect();
    std::pair<double, double> getAngularVelocity() const;
};

#endif // ZLAC8015D_DRIVER_HPP
