#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
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
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override {
        if (configure_default(info) != hardware_interface::return_type::OK) {
            return hardware_interface::return_type::ERROR;
        }

        hw_positions_.resize(info.joints.size(), 0.0);
        hw_velocities_.resize(info.joints.size(), 0.0);
        hw_commands_.resize(info.joints.size(), 0.0);

        serial_port_  = info_.hardware_parameters.at("serial_port");
        baud_rate_    = std::stoi(info_.hardware_parameters.at("baud_rate"));
        wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
        wheel_base_   = std::stod(info_.hardware_parameters.at("wheel_base"));

        return hardware_interface::return_type::OK;
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

    hardware_interface::return_type start() override {
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TestDDBotHardware"), "Unable to open serial port: %s", serial_port_.c_str());
            return hardware_interface::return_type::ERROR;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::return_type::ERROR;
        }

        cfsetispeed(&tty, baud_rate_);
        cfsetospeed(&tty, baud_rate_);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag     = 0;
        tty.c_lflag     = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type stop() override {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type read() override {
        if (fd_ >= 0) {
            std::array<char, 64> buffer{};
            auto bytes_read = read(fd_, buffer.data(), buffer.size());
            (void)bytes_read;
        }

        for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
            hw_positions_[i] += hw_commands_[i] * 0.02;
            hw_velocities_[i] = hw_commands_[i];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write() override {
        if (fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        int left_pwm  = static_cast<int>(std::lround(hw_commands_[0] * 255.0 / 0.6));
        int right_pwm = static_cast<int>(std::lround(hw_commands_[1] * 255.0 / 0.6));
        left_pwm      = std::clamp(left_pwm, -255, 255);
        right_pwm     = std::clamp(right_pwm, -255, 255);

        std::string msg = "m " + std::to_string(left_pwm) + " " + std::to_string(right_pwm) + "\n";
        write(fd_, msg.c_str(), msg.size());
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
