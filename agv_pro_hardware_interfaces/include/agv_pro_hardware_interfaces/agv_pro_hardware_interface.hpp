#ifndef AGV_PRO_HARDWARE_INTERFACE_HPP
#define AGV_PRO_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/macros.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

namespace agv_pro_hardware_interfaces
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Serial communication constants
#define SEND_DATA_SIZE 14                               // Total bytes in a command frame to ESP32(version>=V1.0.8)
#define RECEIVE_FRAME_SIZE 31                           // Total bytes in a frame from ESP32(version>=V1.0.8)
#define RECEIVE_PAYLOAD_SIZE (RECEIVE_FRAME_SIZE - 3)   // Payload length (excluding header)

// Command IDs
#define POWER_ON 0x10
#define GET_POWER_STATE 0x12
#define SET_AUTO_REPORT_STATE 0x23
#define SET_OUTPUT_IO 0x40
#define GET_INPUT_IO 0x41

class AgvProInterface : public hardware_interface::SystemInterface
{
public:
    // LifecycleNodeInterface
    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    
    // SystemInterface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Serial communication
    std::string port_;
    int baudrate_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    boost::asio::io_service io_;

    // Motor state and command storage
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> velocity_commands_;

    // Other hardware states
    float battery_voltage_;
    uint8_t motor_status_;
    uint8_t motor_error_;
    uint8_t enable_status_;

    // Odometry data
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vtheta_ = 0.0;

    // Serial communication methods
    bool send_serial_frame(const std::vector<uint8_t>& frame, bool debug = false);
    std::vector<uint8_t> read_serial_response(const std::vector<uint8_t>& expected_header, size_t payload_size, double timeout_sec);
    uint16_t crc16_ibm(const uint8_t* data, size_t length);
    std::vector<uint8_t> build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload);
    bool readData();
    bool is_power_on();
    void set_auto_report(bool enable);
    void clearSerialBuffer(int fd);
    void disableDTR_RTS(int fd);
    void print_hex(const std::string& label, const std::vector<uint8_t>& data, std::optional<size_t> override_size = std::nullopt);
};

} // namespace agv_pro_hardware_interfaces

#endif // AGV_PRO_HARDWARE_INTERFACE_HPP
