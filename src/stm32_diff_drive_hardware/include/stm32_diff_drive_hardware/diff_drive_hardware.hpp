#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>
#include <string>
#include <vector>

namespace stm32_diff_drive_hardware
{

class STM32DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
    STM32DiffDriveHardware();

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    
    // -- VECTORS CỦA ROS2 CONTROL --
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    
    // --- BIẾN CẦN THIẾT CHO LOGIC I/O VÀ VẬN TỐC ---
    /**
     * Buffer để lưu trữ dữ liệu serial chưa hoàn chỉnh (FIX Blocking I/O)
     */
    std::string serial_read_buffer_; 
    
    /**
     * Lưu trữ vị trí của chu kỳ trước đó để tính toán vận tốc hiện tại.
     */
    std::vector<double> hw_prev_positions_; 
    
    // hardware params
    std::string port_;
    int baudrate_;
    double wheel_radius_;
    int enc_counts_per_rev_;
    double dist_per_tick_ = 0.0;
    double rad_per_tick_;
    int last_tick_left_ = 0;
    int last_tick_right_ = 0;
    bool first_read_ = true;
    // Các biến riêng lẻ (Giữ nguyên từ code gốc)
    double left_position_ = 0.0;
    double left_velocity_ = 0.0;
    double left_command_ = 0.0;

    double right_position_ = 0.0;
    double right_velocity_ = 0.0;
    double right_command_ = 0.0;
};

} // namespace stm32_diff_drive_hardware