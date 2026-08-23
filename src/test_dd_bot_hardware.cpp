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
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);


        serial_port_  = info_.hardware_parameters["serial_port"];
        baud_rate_    = std::stoi(info_.hardware_parameters["baud_rate"]);
        wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
        wheel_base_   = std::stod(info_.hardware_parameters["wheel_base"]);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
        }
        return command_interfaces;
    }

    speed_t map_baud_rate(int baud) {
        switch (baud) {
            case 1200: return B1200;
            case 1800: return B1800;
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            default: return B9600;
        }
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Unable to open serial port: %s", serial_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }

        speed_t baud = map_baud_rate(baud_rate_);
        if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
            close(fd_);
            fd_ = -1;
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Failed to set serial baud rate: %d", baud_rate_);
            return hardware_interface::CallbackReturn::ERROR;
        }
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag     = 0;
        tty.c_lflag     = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;  // 0.1 second timeout
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        if (fd_ >= 0) {
            std::array<char, 64> buffer{};
            auto bytes_read = ::read(fd_, buffer.data(), buffer.size());
            if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_WARN(rclcpp::get_logger("TestDDBotHardware"), "Serial read failed: %s", std::strerror(errno));
            }
        }

        for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
            hw_positions_[i] += hw_commands_[i] * 0.02;
            hw_velocities_[i] = hw_commands_[i];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        if (fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        constexpr double MAX_WHEEL_SPEED = 9.23;   // rad/s
        constexpr double MAX_PWM = 255.0;

        int left_pwm  = static_cast<int>(std::lround(hw_commands_[0] * MAX_PWM / MAX_WHEEL_SPEED));
        int right_pwm = static_cast<int>(std::lround(hw_commands_[1] * MAX_PWM / MAX_WHEEL_SPEED));
        left_pwm      = std::clamp(left_pwm, -255, 255);
        right_pwm     = std::clamp(right_pwm, -255, 255);

        std::ostringstream ss;
        ss << "o "
            << left_pwm
            << " "
            << right_pwm
            << "\r";
        std::string msg = ss.str();

        auto bytes_written = ::write(fd_, msg.c_str(), msg.size());
        if (bytes_written < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Serial write failed: %s", std::strerror(errno));
            return hardware_interface::return_type::ERROR;
        }
        if (static_cast<size_t>(bytes_written) != msg.size()) {
            RCLCPP_WARN(rclcpp::get_logger("TestDDBotHardware"), "Partial serial write: %zd / %zu bytes", bytes_written, msg.size());
        }
        static int write_count = 0;
        if ((++write_count % 10) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("TestDDBotHardware"), "serial out: %s", msg.c_str());
        }
        return hardware_interface::return_type::OK;
    }

  private:
    int fd_{-1};
    std::string serial_port_;
    int baud_rate_{9600};
    double wheel_radius_{0.065};
    double wheel_base_{0.34};
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};

} // namespace test_dd_bot_hardware

PLUGINLIB_EXPORT_CLASS(test_dd_bot_hardware::TestDDBotHardware, hardware_interface::SystemInterface)
