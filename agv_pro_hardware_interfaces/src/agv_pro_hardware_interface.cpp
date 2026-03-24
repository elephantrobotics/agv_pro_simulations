#include "agv_pro_hardware_interfaces/agv_pro_hardware_interface.hpp"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace agv_pro_hardware_interfaces
{

CallbackReturn AgvProInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Get serial port parameters from hardware info
    port_ = info_.hardware_parameters.at("port");
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

    RCLCPP_INFO(
        rclcpp::get_logger("AgvProInterface"),
        "Using port=%s baudrate=%d", port_.c_str(), baudrate_);

    // Check joint interfaces
    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("AgvProInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.",
                        joint.name.c_str(), joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AgvProInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("AgvProInterface"),
                        "Joint '%s' has %zu state interface. 2 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AgvProInterface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AgvProInterface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }
    }

    // Initialize state and command vectors
    position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    velocity_commands_.resize(info_.joints.size(), 0.0);

    // Initialize other states
    battery_voltage_ = 0.0f;
    motor_status_ = 0;
    motor_error_ = 0;
    enable_status_ = 0;

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_configure(const rclcpp_lifecycle::State& )
{
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Configuring AGV Pro hardware interface");

    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_);
        serial_port_->open(port_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        int fd = serial_port_->native_handle();
        clearSerialBuffer(fd);
        disableDTR_RTS(fd);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); // esp32 Restart time

        RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Serial port initialized successfully");
        RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Using device: %s", port_.c_str());
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Failed to initialize serial port: %s", ex.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_activate(const rclcpp_lifecycle::State& )
{
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Activating AGV Pro hardware interface");

    if (!is_power_on()) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Failed to power on AGV Pro");
        return CallbackReturn::ERROR;
    }

    set_auto_report(true);
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "AGV Pro hardware interface activated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_deactivate(const rclcpp_lifecycle::State& )
{
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Deactivating AGV Pro hardware interface");

    set_auto_report(false);
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "AGV Pro hardware interface deactivated");

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_shutdown(const rclcpp_lifecycle::State& )
{
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Shutting down AGV Pro hardware interface");

    if (serial_port_ && serial_port_->is_open()) {
        set_auto_report(false);
        serial_port_->cancel();
        serial_port_->close();
        RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Serial port closed");
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_cleanup(const rclcpp_lifecycle::State& )
{
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Cleaning up AGV Pro hardware interface");

    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn AgvProInterface::on_error(const rclcpp_lifecycle::State& )
{
    RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Error in AGV Pro hardware interface");

    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }

    return CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> AgvProInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_states_[i]
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_states_[i]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AgvProInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_commands_[i]
            )
        );
    }

    return command_interfaces;
}

hardware_interface::return_type AgvProInterface::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
    // Read data from AGV
    if (!readData())
    {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Failed to read data from AGV");
    }

    // Update joint states based on odometry
    const double wheel_radius = 0.076;       // 轮子半径
    const double wheel_base = 0.312 / 2;     // 左右轮距半值 = 0.156 米
    const double wheel_track = 0.3431 / 2;   // 前后轮距半值 = 0.17155 米

    for (size_t i = 0; i < velocity_states_.size(); ++i) {

        switch(i) {
            case 0: // 左前轮 (FL)
                velocity_states_[i] = (vx_ - vy_ - (wheel_base + wheel_track) * vtheta_) / wheel_radius;
                break;
            case 1: // 右前轮 (FR)
                velocity_states_[i] = (vx_ + vy_ + (wheel_base + wheel_track) * vtheta_) / wheel_radius;
                break;
            case 2: // 左后轮 (RL)
                velocity_states_[i] = (vx_ + vy_ - (wheel_base + wheel_track) * vtheta_) / wheel_radius;
                break;
            case 3: // 右后轮 (RR)
                velocity_states_[i] = (vx_ - vy_ + (wheel_base + wheel_track) * vtheta_) / wheel_radius;
                break;
            default: // 防呆：超出4轮时速度置0
                velocity_states_[i] = 0.0;
                break;
        }

        position_states_[i] += velocity_states_[i] * period.seconds();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvProInterface::write(const rclcpp::Time&, const rclcpp::Duration& )
{
    // Build and send velocity command frame
    std::vector<uint8_t> payload;
    
    // Calculate linear and angular velocity from wheel commands
    // Mecanum drive kinematics
    double linear_x = (velocity_commands_[0] + velocity_commands_[1] + velocity_commands_[3] + velocity_commands_[2]) / 4.0;
    double linear_y = (-velocity_commands_[0] + velocity_commands_[1] + velocity_commands_[3] - velocity_commands_[2]) / 4.0;
    double angular_z = (-velocity_commands_[0] + velocity_commands_[1] - velocity_commands_[3] + velocity_commands_[2]) / 4.0;
    
    double linear_scale = 0.1;  // Further reduce linear speed
    double angular_scale = 0.18; // Further reduce angular speed

    linear_x *= linear_scale;
    linear_y *= linear_scale;
    angular_z *= angular_scale;
    
    // Clamp values to AGV Pro's limits
    linear_x = std::clamp(linear_x, -1.5, 1.5);
    linear_y = std::clamp(linear_y, -1.0, 1.0);
    angular_z = std::clamp(angular_z, -1.0, 1.0);
    
    // Convert to AGV Pro's command format
    int16_t x_send = static_cast<int16_t>(linear_x * 100);
    int16_t y_send = static_cast<int16_t>(linear_y * 100);
    int16_t rot_send = static_cast<int16_t>(angular_z * 100);
    
    // Build payload
    payload.push_back((x_send >> 8) & 0xff);
    payload.push_back(x_send & 0xff);
    payload.push_back((y_send >> 8) & 0xff);
    payload.push_back(y_send & 0xff);
    payload.push_back((rot_send >> 8) & 0xff);
    payload.push_back(rot_send & 0xff);
    payload.push_back(0x00);
    payload.push_back(0x00);

    // Build and send frame
    auto frame = build_serial_frame(0x21, payload); // 0x21 is velocity command for AGV Pro
    send_serial_frame(frame);

    return hardware_interface::return_type::OK;
}

uint16_t AgvProInterface::crc16_ibm(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

std::vector<uint8_t> AgvProInterface::build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame(SEND_DATA_SIZE, 0x00);
    frame[0] = 0xFE;
    frame[1] = 0xFE;
    frame[2] = 0x0B;
    frame[3] = cmd_id;

    for (size_t i = 0; i < payload.size() && i < 8; ++i) {
        frame[4 + i] = payload[i];
    }

    uint16_t crc = crc16_ibm(frame.data(), 12);
    frame[12] = (crc >> 8) & 0xff;
    frame[13] = crc & 0xff;

    return frame;
}

void AgvProInterface::print_hex(const std::string& label, const std::vector<uint8_t>& data, std::optional<size_t> override_size)
{
    std::stringstream ss;
    for (auto b : data) {
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
           << static_cast<int>(b) << " ";
    }
    size_t len = override_size.value_or(data.size());
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "%s (%zu bytes): [%s]", label.c_str(), len, ss.str().c_str());
}

bool AgvProInterface::send_serial_frame(const std::vector<uint8_t>& frame, bool debug)
{
    try {
        size_t bytes_transmit_size = boost::asio::write(*serial_port_, boost::asio::buffer(frame));
        if (debug) {
            print_hex("Sent", frame, bytes_transmit_size);
        }
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Error Transmiting from serial port: %s", ex.what());
        return false;
    }
}

std::vector<uint8_t> AgvProInterface::read_serial_response(
    const std::vector<uint8_t>& expected_header,
    size_t payload_size,
    double timeout_sec)
{
    std::vector<uint8_t> sliding_buf;
    uint8_t byte = 0;

    rclcpp::Time start_time = rclcpp::Clock().now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);

    while ((rclcpp::Clock().now() - start_time) < timeout) {
        boost::asio::mutable_buffers_1 buf(&byte, 1);
        boost::system::error_code ec;
        size_t n = serial_port_->read_some(buf, ec);
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Serial read error: %s", ec.message().c_str());
            return {};
        }
        if (n == 1) {
            sliding_buf.push_back(byte);
            if (sliding_buf.size() > expected_header.size()) {
                sliding_buf.erase(sliding_buf.begin());
            }
            if (sliding_buf == expected_header) {
                break;
            }
        }
    }
    
    if (sliding_buf != expected_header) {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Timeout waiting for header");
        return {};
    }

    size_t remain_len = payload_size + 2;
    std::vector<uint8_t> remain_buf(remain_len);
    size_t total_read = 0;

    while (total_read < remain_len && (rclcpp::Clock().now() - start_time) < timeout) {
        boost::asio::mutable_buffers_1 buf(&remain_buf[total_read], remain_len - total_read);
        boost::system::error_code ec;
        size_t n = serial_port_->read_some(buf, ec);
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Serial read error: %s", ec.message().c_str());
            return {};
        }
        total_read += n;
    }

    if (total_read != remain_len) {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Timeout or incomplete data payload");
        return {};
    }

    std::vector<uint8_t> full_buf = expected_header;
    full_buf.insert(full_buf.end(), remain_buf.begin(), remain_buf.end());

    return full_buf;
}

bool AgvProInterface::is_power_on()
{
    auto power_query_frame = build_serial_frame(GET_POWER_STATE, {});
    send_serial_frame(power_query_frame, true);

    const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, 0x12};
    auto power_query_response = read_serial_response(expected_header, 8, 12.0);

    print_hex("recv_buf", power_query_response);
    
    if (power_query_response.size() != 14) return false;

    uint16_t received_crc = (power_query_response[12] << 8) | power_query_response[13];
    uint16_t computed_crc = crc16_ibm(power_query_response.data(), 12);
    if (received_crc != computed_crc) {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "CRC mismatch: received=0x%04X, expected=0x%04X", received_crc, computed_crc);
        return false;
    }

    int is_poweron_status = static_cast<int8_t>(power_query_response[4]);
    RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "is_poweron_status: %d", is_poweron_status);

    if (is_poweron_status == 0) {
        auto status_query_frame = build_serial_frame(POWER_ON, {});
        send_serial_frame(status_query_frame, true);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, 0x10};
        auto status_query_response = read_serial_response(expected_header, 8, 5.0);
        print_hex("recv_buf", status_query_response);
    
        if (status_query_response.size() != 14) return false;

        uint16_t received_crc = (status_query_response[12] << 8) | status_query_response[13];
        uint16_t computed_crc = crc16_ibm(status_query_response.data(), 12);
        if (received_crc != computed_crc) {
            RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "CRC mismatch: received=0x%04X, expected=0x%04X", received_crc, computed_crc);
            return false;
        }

        int poweron_status = static_cast<int8_t>(status_query_response[4]);
        std::string status_msg;

        switch (poweron_status) {
            case 1:
                status_msg = "Motor is operating normally.";
                RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "power_status: %d, %s", poweron_status, status_msg.c_str());
                return true;
            case 2:
                status_msg = "Emergency stop button is not released.";
                RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "power_status: %d, %s", poweron_status, status_msg.c_str());
                return false;
            case 3:
                status_msg = "Battery voltage is below 19.5V.";
                RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "power_status: %d, %s", poweron_status, status_msg.c_str());
                return false;
            case 4:
                status_msg = "CAN initialization error.";
                RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "power_status: %d, %s", poweron_status, status_msg.c_str());
                return false;
            case 5:
                status_msg = "Motor initialization error.";
                RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "power_status: %d, %s", poweron_status, status_msg.c_str());
                return false;
            default:
                RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "power_status: %d, Unknown power status code", poweron_status);
                return false;
        }
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Motor is operating normally.");
        return true;
    }
}

void AgvProInterface::set_auto_report(bool enable)
{
    auto frame = build_serial_frame(SET_AUTO_REPORT_STATE, {static_cast<uint8_t>(enable)});
    send_serial_frame(frame, true);
}

void AgvProInterface::clearSerialBuffer(int fd)
{
    if (tcflush(fd, TCIOFLUSH) < 0) {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Failed to flush serial buffer: %s", std::strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "Serial buffer flushed.");
    }
}

void AgvProInterface::disableDTR_RTS(int fd)
{
    int status;
    if (::ioctl(fd, TIOCMGET, &status) == 0) {
        status &= ~(TIOCM_DTR | TIOCM_RTS);
        if (::ioctl(fd, TIOCMSET, &status) != 0) {
            RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Failed to clear DTR and RTS: %s", std::strerror(errno));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("AgvProInterface"), "DTR and RTS lines disabled successfully.");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Failed to read modem status: %s", std::strerror(errno));
    }
}

bool AgvProInterface::readData()
{
    std::vector<uint8_t> buf_length(1);
    std::vector<uint8_t> data_buf;

    uint8_t byte = 0;
    boost::system::error_code ec;
    
    while (true) {
        size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Serial read error: %s", ec.message().c_str());
            return false;
        }
        if (ret != 1 || byte != 0xfe) {
            continue;
        }

        ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Serial read error: %s", ec.message().c_str());
            return false;
        }
        if (ret == 1 && byte == 0xfe) {
            break; 
        }
    }

    size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(buf_length), ec);
    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Serial read error: %s", ec.message().c_str());
        return false;
    }

    // 读取数据 payload
    data_buf.resize(buf_length[0]);
    ret = boost::asio::read(*serial_port_, boost::asio::buffer(data_buf), ec);
    if (ec || ret != data_buf.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvProInterface"), "Failed to receive full payload");
        return false;
    }

    // 构建完整数据帧
    std::vector<uint8_t> recv_buf;
    recv_buf.push_back(0xFE);
    recv_buf.push_back(0xFE);
    recv_buf.push_back(buf_length[0]);
    recv_buf.insert(recv_buf.end(), data_buf.begin(), data_buf.end());
    
    // 检查命令ID
    if (recv_buf[3] != 0x25 && recv_buf[3] != 0x21 && recv_buf[3] != 0x23) {
        RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "Invalid cmd ID: 0x%02X (expected 0x25, 0x21, or 0x23)", recv_buf[3]);
        return false;
    }

    // 计算和验证CRC
    if (recv_buf.size() < 4) {
        return false;
    }
    
    // 对于0x21命令（速度命令确认），只需要简单确认
    if (recv_buf[3] == 0x21) {
        RCLCPP_DEBUG(rclcpp::get_logger("AgvProInterface"), "Received speed command acknowledgment");
        return true;
    }

    // 对于0x23命令（自动报告状态设置响应），只需要简单确认
    if (recv_buf[3] == 0x23) {
        RCLCPP_DEBUG(rclcpp::get_logger("AgvProInterface"), "Received auto report status response");
        return true;
    }

    // 对于0x25命令（自动报告），需要完整处理
    if (recv_buf[3] == 0x25) {
        // 检查CRC
        if (recv_buf.size() < 6) {
            return false;
        }
        
        size_t crc_start = recv_buf.size() - 2;
        uint16_t received_crc = (recv_buf[crc_start] << 8) | recv_buf[crc_start + 1];
        uint16_t computed_crc = crc16_ibm(recv_buf.data(), crc_start);

        if (received_crc != computed_crc) {
            RCLCPP_WARN(rclcpp::get_logger("AgvProInterface"), "CRC error: received 0x%04X, calculated 0x%04X", received_crc, computed_crc);
            return false;
        }

        // 解析数据
        if (recv_buf.size() >= 14) {
            vx_ = static_cast<double>(static_cast<int8_t>(recv_buf[4])) * 0.01;
            vy_ = static_cast<double>(static_cast<int8_t>(recv_buf[5])) * 0.01;
            vtheta_ = static_cast<double>(static_cast<int8_t>(recv_buf[6])) * 0.01;

            motor_status_ = recv_buf[7];
            motor_error_ = recv_buf[8];
            battery_voltage_ = static_cast<float>(recv_buf[9]) / 10.0f;
            enable_status_ = recv_buf[10];
        }

        return true;
    }

    return false;
}

} // namespace agv_pro_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agv_pro_hardware_interfaces::AgvProInterface,  // 注意：是 AgvProInterface
  hardware_interface::SystemInterface
)