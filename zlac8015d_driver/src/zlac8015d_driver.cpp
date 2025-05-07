#include "zlac8015d_driver/zlac8015d_driver.hpp"
#include <modbus/modbus.h>
#include <unistd.h>
#include <ctime>

// Địa chỉ thanh ghi Modbus của ZLAC8015D
#define REG_CONTROL 0x200E
#define REG_CMD_RPM_LEFT 0x2088
#define REG_CMD_RPM_RIGHT 0x2089
#define REG_FB_RPM_LEFT 0x20AB
#define REG_FB_RPM_RIGHT 0x20AC
#define REG_FAULT_LEFT 0x20A5
#define REG_FAULT_RIGHT 0x20A6
#define REG_WHEEL_POS 0x20A7
#define REG_BATTERY_VOLTAGE 0x20A1
#define REG_MOTOR_TEMP 0x20A4
#define REG_DRIVER_TEMP 0x20B0

#define L_ACL_TIME 0x2080
#define R_ACL_TIME 0x2081
#define CLR_FB_POS 0x2005

// Giá trị điều khiển
#define CMD_ENABLE 0x08
#define CMD_DISABLE 0x07
#define CMD_EMERGENCY_STOP 0x05

ZLAC8015D_API::ZLAC8015D_API(const std::string &port, int baudrate, int slave_id)
{
    this->ctx = modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1);
    this->slave_id = slave_id;
    this->was_connected = false;
    this->reconnect_interval = 5;
    this->last_attempt_time = std::time(nullptr);

    if (ctx == nullptr)
    {
        std::cerr << "❌ Không thể khởi tạo Modbus RTU!\n";
        return;
    }

    modbus_set_slave(ctx, slave_id);
    modbus_set_response_timeout(ctx, 1, 0); // 1s timeout

    if (modbus_connect(ctx) == -1)
    {
        std::cerr << "⚠️ Kết nối Modbus thất bại!\n";
        modbus_free(ctx);
        ctx = nullptr;
    }
    else
    {
        std::cout << "✅ Kết nối Modbus thành công!\n";
        was_connected = true;
    }
}

bool ZLAC8015D_API::isConnected()
{
    if (!ctx)
        return false;
    return modbus_connect(ctx) == 0;
}

ZLAC8015D_API::~ZLAC8015D_API()
{
    if (ctx)
    {
        modbus_close(ctx);
        modbus_free(ctx);
    }
}

bool ZLAC8015D_API::checkConnection()
{
    if (!ctx)
        return false;
    std::vector<uint16_t> result = readRegisters(REG_FAULT_LEFT, 1);
    return !result.empty();
}

bool ZLAC8015D_API::reconnect()
{
    double now = std::time(nullptr);
    if (was_connected && (now - last_attempt_time < reconnect_interval))
        return false;

    last_attempt_time = now;
    std::cerr << "⚠️ Mất kết nối Modbus! Đang thử kết nối lại...\n";

    if (modbus_connect(ctx) == -1)
    {
        std::cerr << "❌ Không thể kết nối lại Modbus!\n";
        return false;
    }

    std::cout << "✅ Kết nối lại Modbus thành công!\n";
    was_connected = true;
    return true;
}

std::vector<uint16_t> ZLAC8015D_API::readRegisters(int addr, int num) const
{
    std::vector<uint16_t> values(num);
    if (modbus_read_registers(ctx, addr, num, values.data()) == num)
    {
        return values;
    }
    return {};
}

bool ZLAC8015D_API::writeRegister(int addr, uint16_t value)
{
    return modbus_write_register(ctx, addr, value) != -1;
}

bool ZLAC8015D_API::writeRegisters(int addr, const std::vector<uint16_t> &values)
{
    return modbus_write_registers(ctx, addr, values.size(), values.data()) != -1;
}

bool ZLAC8015D_API::enableMotor()
{
    return writeRegister(REG_CONTROL, CMD_ENABLE);
}

bool ZLAC8015D_API::disableMotor()
{
    return writeRegister(REG_CONTROL, CMD_DISABLE);
}

bool ZLAC8015D_API::emergencyStop()
{
    return writeRegister(REG_CONTROL, CMD_EMERGENCY_STOP);
}

bool ZLAC8015D_API::setRPM(int left_rpm, int right_rpm)
{

    // left_rpm = std::clamp(left_rpm, -200, 200);
    // right_rpm = std::clamp(right_rpm, -200, 200);

    left_rpm = std::max(-200, std::min(200, left_rpm));
    right_rpm = std::max(-200, std::min(200, right_rpm));
    return writeRegisters(REG_CMD_RPM_LEFT, {static_cast<uint16_t>(left_rpm), static_cast<uint16_t>(right_rpm)});
}

std::pair<int, int> ZLAC8015D_API::getRPM() const
{
    std::vector<uint16_t> regs = readRegisters(REG_FB_RPM_LEFT, 2);
    if (regs.size() < 2)
        return {0, 0};
    return {static_cast<int>(regs[0]) / 10, static_cast<int>(regs[1]) / 10};
}

std::pair<uint16_t, uint16_t> ZLAC8015D_API::getMotorFaults()
{
    std::vector<uint16_t> regs = readRegisters(REG_FAULT_LEFT, 2);
    return {regs.size() >= 2 ? regs[0] : 0, regs.size() >= 2 ? regs[1] : 0};
}

std::pair<double, double> ZLAC8015D_API::getWheelsTravelled()
{
    std::vector<uint16_t> regs = readRegisters(REG_WHEEL_POS, 4);
    if (regs.size() < 4)
        return std::make_pair(0.0, 0.0);

    return std::make_pair(
        static_cast<double>((regs[0] << 16) | regs[1]),
        static_cast<double>((regs[2] << 16) | regs[3]));
}


// std::array<double, 2> ZLAC8015D_API::getWheelsTravelled() const
// {
//     std::vector<uint16_t> regs = readRegisters(REG_WHEEL_POS, 4);
//     if (regs.size() < 4)
//         return {0.0, 0.0};

//     return {static_cast<double>((regs[0] << 16) | regs[1]),
//             static_cast<double>((regs[2] << 16) | regs[3])};
// }


std::pair<int32_t, int32_t> ZLAC8015D_API::getWheelsTick()
{
    auto travelled = getWheelsTravelled();
    return {static_cast<int32_t>(travelled.first), static_cast<int32_t>(travelled.second)};
}

int ZLAC8015D_API::getMotorTemperature()
{
    std::vector<uint16_t> regs = readRegisters(REG_MOTOR_TEMP, 1);
    return regs.empty() ? 0 : regs[0];
}

int ZLAC8015D_API::getDriverTemperature()
{
    std::vector<uint16_t> regs = readRegisters(REG_DRIVER_TEMP, 1);
    return regs.empty() ? 0 : regs[0] * 0.1;
}

double ZLAC8015D_API::getBatteryVoltage()
{
    std::vector<uint16_t> regs = readRegisters(REG_BATTERY_VOLTAGE, 1);
    return regs.empty() ? 0.0 : regs[0] * 0.01;
}
void ZLAC8015D_API::setDecelTime(int left_ms, int right_ms)
{
    uint16_t values[2] = {static_cast<uint16_t>(left_ms), static_cast<uint16_t>(right_ms)};
    modbus_write_registers(ctx, R_ACL_TIME, 2, values);
}

void ZLAC8015D_API::setAccelTime(int left_ms, int right_ms)
{
    uint16_t values[2] = {static_cast<uint16_t>(left_ms), static_cast<uint16_t>(right_ms)};
    modbus_write_registers(ctx, L_ACL_TIME, 2, values);
}
void ZLAC8015D_API::clear_position(int pos)
{
    modbus_write_register(ctx, CLR_FB_POS, pos);
}
void ZLAC8015D_API::close_connect()
{
    if (ctx)
    {
        modbus_close(ctx);
        modbus_free(ctx);
    }
}

std::pair<double, double> ZLAC8015D_API::getAngularVelocity() const
{
    // Gọi hàm getRPM() để lấy tốc độ RPM của bánh trái và bánh phải
    std::pair<int, int> rpm = getRPM();

    // Chuyển đổi RPM thành vận tốc góc (rad/s)
    double left_rpm = rpm.first * 2 * M_PI / 60.0;
    double right_rpm = rpm.second * 2 * M_PI / 60.0;

    return std::make_pair(left_rpm, right_rpm);
}