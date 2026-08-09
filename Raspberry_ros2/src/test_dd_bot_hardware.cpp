#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace test_dd_bot_hardware {
class TestDDBotHardware : public hardware_interface::SystemInterface {
  public:
    // Called once when the hardware plugin is loaded by ros2 control.
    // Reads URDF file and ros2_control tag parameters and allocates state/command buffers.
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override {
        // Always call the base class on_init first; abort if it fails.
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Allocate one entry per joint for position, velocity, and command buffers.
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        // Read custom hardware parameters defined in the ros2_control tag in URDF file.
        serial_port_  = info_.hardware_parameters["serial_port"];
        baud_rate_    = std::stoi(info_.hardware_parameters["baud_rate"]);
        wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
        wheel_base_   = std::stod(info_.hardware_parameters["wheel_base"]);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Exposes readable state interfaces (position, velocity) for each joint
    // so controllers (e.g. joint_state_broadcaster) can read them.
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        return state_interfaces;
    }

    // Exposes writable command interfaces (velocity) for each joint
    // so controllers (e.g. diff_drive_controller) can send commands.
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
        }
        return command_interfaces;
    }

    // Maps an integer baud rate (e.g. 9600) to the termios speed_t constant.
    // Falls back to B9600 if the value is not recognized.
    speed_t map_baud_rate(int baud) {
        switch (baud) {
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        default:
            return B9600;
        }
    }

    // Called when the hardware component transitions to "active".
    // Opens and configures the serial port (raw mode, no flow control, 8N1).
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        // Open the serial device in read/write, non-blocking mode.
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Unable to open serial port: %s", serial_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Read current terminal attributes so we can modify them.
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Set input/output baud rate based on the configured parameter.
        speed_t baud = map_baud_rate(baud_rate_);
        if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
            close(fd_);
            fd_ = -1;
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Failed to set serial baud rate: %d", baud_rate_);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure for 8 data bits, no parity, 1 stop bit (8N1), raw mode.
        tty.c_cflag     = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
        tty.c_iflag&    = ~(IXON | IXOFF | IXANY);      // disable software flow control
        tty.c_oflag     = 0;                            // no output processing (raw)
        tty.c_lflag     = 0;                            // no canonical mode, no echo, no signals (raw)
        tty.c_cc[VMIN]  = 0;                            // read() returns immediately if no data...
        tty.c_cc[VTIME] = 1;                            // ...after waiting up to 0.1 second timeout
        tty.c_cflag |   = (CLOCAL | CREAD);             // enable receiver, ignore modem control lines
        tty.c_cflag&    = ~(PARENB | PARODD);           // no parity
        tty.c_cflag&    = ~CSTOPB;                      // 1 stop bit
        tty.c_cflag&    = ~CRTSCTS;                     // disable hardware flow control (RTS/CTS)

        // Apply the new settings immediately.
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Called when the hardware component transitions out of "active".
    // Closes the serial port if it's open.
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Called every control loop cycle (realtime thread) to update joint state.
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override {
        // Check serial port.
        if (fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        // Request encoder values from Arduino.
        // Arduino responds: "left_ticks right_ticks\r"
        constexpr char ENCODER_COMMAND[] = "e\r";

        const ssize_t command_bytes = ::write(fd_, ENCODER_COMMAND, sizeof(ENCODER_COMMAND) - 1);

        if (command_bytes < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"),
                         "Failed to request encoder data: %s",
                         std::strerror(errno));

            return hardware_interface::return_type::ERROR;
        }

        // Store incomplete serial data between read() calls.
        static std::string rx_buffer;
        std::array<char, 128> buffer{};

        // Read all available serial data.
        while (true) {
            const ssize_t bytes_read = ::read(fd_, buffer.data(), buffer.size());

            // No more data available.
            if (bytes_read < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }

                RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"),
                             "Serial read failed: %s",
                             std::strerror(errno));

                return hardware_interface::return_type::ERROR;
            }

            // No data received.
            if (bytes_read == 0) {
                break;
            }

            // Append received data to the buffer.
            rx_buffer.append(buffer.data(), static_cast<std::size_t>(bytes_read));
        }

        // Wait until a complete packet is received.
        const std::size_t end_pos = rx_buffer.find('\r');

        if (end_pos == std::string::npos) {
            return hardware_interface::return_type::OK;
        }

        // Extract one complete packet.
        const std::string packet = rx_buffer.substr(0, end_pos);

        // Remove the processed packet.
        rx_buffer.erase(0, end_pos + 1);

        // Parse: "left_ticks right_ticks"
        long long left_ticks = 0;
        long long right_ticks = 0;

        std::istringstream iss(packet);

        if (!(iss >> left_ticks >> right_ticks)) {

            RCLCPP_WARN(rclcpp::get_logger("TestDDBotHardware"),
                         "Invalid encoder packet: '%s'",
                         packet.c_str());

            return hardware_interface::return_type::OK;
        }

        // Encoder resolution.
        constexpr double ENCODER_TICKS_PER_REV = 600.0;

        // Convert encoder ticks to radians.
        const double left_position = static_cast<double>(left_ticks) * (2.0 * M_PI / ENCODER_TICKS_PER_REV);
        const double right_position = static_cast<double>(right_ticks) * (2.0 * M_PI / ENCODER_TICKS_PER_REV);

        // Calculate wheel velocity: velocity = Δposition / Δtime.
        const double dt = period.seconds();

        if (dt > 0.0) {
            hw_velocities_[0] = (left_position - hw_positions_[0]) / dt;
            hw_velocities_[1] = (right_position - hw_positions_[1]) / dt;
        }

        // Update wheel positions.
        hw_positions_[0] = left_position;
        hw_positions_[1] = right_position;

        // Print encoder data every 50 cycles.
        static int read_count = 0;

        if (++read_count % 50 == 0) {

            RCLCPP_INFO(rclcpp::get_logger("TestDDBotHardware"),
                         "Encoder: L=%lld ticks, R=%lld ticks | "
                         "Position: L=%.3f rad, R=%.3f rad | "
                         "Velocity: L=%.3f rad/s, R=%.3f rad/s",
                         left_ticks,
                         right_ticks,
                         hw_positions_[0],
                         hw_positions_[1],
                         hw_velocities_[0],
                         hw_velocities_[1]);
        }

        return hardware_interface::return_type::OK;
    }

    // Called every control loop cycle (realtime thread) to send commands to hardware.
    // Converts commanded wheel velocity (rad/s) into a PWM value and writes it
    // out over serial as a simple text protocol: "m <left_pwm> <right_pwm>\r".
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        if (fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        constexpr double MAX_WHEEL_SPEED = 9.23;  // max wheel speed in rad/s (motor spec)
        constexpr double MAX_PWM         = 255.0; // 8-bit PWM range

        // Scale commanded velocity (rad/s) linearly to PWM duty cycle.
        int left_pwm  = static_cast<int>(std::lround(hw_commands_[0] * MAX_PWM / MAX_WHEEL_SPEED));
        int right_pwm = static_cast<int>(std::lround(hw_commands_[1] * MAX_PWM / MAX_WHEEL_SPEED));
        // Clamp to valid signed PWM range in case commanded velocity exceeds max.
        left_pwm  = std::clamp(left_pwm, -255, 255);
        right_pwm = std::clamp(right_pwm, -255, 255);

        // Build the serial command string, e.g. "m 120 -80\r"
        std::ostringstream ss;
        ss << "m "
           << left_pwm
           << " "
           << right_pwm
           << "\r";
        std::string msg = ss.str();

        // Send the command over serial.
        auto bytes_written = ::write(fd_, msg.c_str(), msg.size());
        if (bytes_written < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Serial write failed: %s", std::strerror(errno));
            return hardware_interface::return_type::ERROR;
        }
        if (static_cast<size_t>(bytes_written) != msg.size()) {
            RCLCPP_WARN(rclcpp::get_logger("TestDDBotHardware"), "Partial serial write: %zd / %zu bytes", bytes_written, msg.size());
        }

        // Throttle logging: only print every 10th write to avoid flooding the console
        // (this runs at the control loop rate, e.g. 50 Hz).
        static int write_count = 0;
        if ((++write_count % 10) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("TestDDBotHardware"), "serial out: %s", msg.c_str());
        }
        return hardware_interface::return_type::OK;
    }

  private:
    int fd_{-1};                        // serial port file descriptor, -1 = closed
    std::string serial_port_;           // e.g. "/dev/ttyUSB0"
    int baud_rate_{9600};
    double wheel_radius_{0.065};        // meters (currently unused in this file)
    double wheel_base_{0.34};           // meters, distance between wheels (currently unused in this file)
    std::vector<double> hw_commands_;   // commanded velocity per joint (rad/s)
    std::vector<double> hw_positions_;  // integrated position per joint (rad)
    std::vector<double> hw_velocities_; // last commanded velocity, reported as feedback (rad/s)
};

} // namespace test_dd_bot_hardware

// Registers this class as a pluginlib-loadable hardware_interface::SystemInterface plugin.
PLUGINLIB_EXPORT_CLASS(test_dd_bot_hardware::TestDDBotHardware, hardware_interface::SystemInterface)