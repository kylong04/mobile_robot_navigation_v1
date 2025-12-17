#include "stm32_diff_drive_hardware/diff_drive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm> 
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sstream>
#include <string>

#include "pluginlib/class_list_macros.hpp"

// --- THƯ VIỆN CẦN THIẾT CHO IOCTL (Linux/Unix) ---
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <fcntl.h>
// ----------------------------------------------------

namespace stm32_diff_drive_hardware
{

// --- HÀM HỖ TRỢ: TRIỂN KHAI serial_.available() bằng ioctl ---
// Sử dụng ioctl trên Linux để kiểm tra số byte có sẵn trong buffer serial.
size_t get_bytes_available(boost::asio::serial_port& serial_port)
{
    int fd = serial_port.native_handle();
    int bytes = 0;
    
    // FIONREAD là cờ để đọc số byte đang chờ trong buffer
    if (ioctl(fd, FIONREAD, &bytes) == -1) {
        return 0;
    }
    return static_cast<size_t>(bytes);
}
// ------------------------------------------------------------------

STM32DiffDriveHardware::STM32DiffDriveHardware()
: serial_(io_)
{}

hardware_interface::CallbackReturn STM32DiffDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_.resize(info.joints.size(), 0.0);
    hw_positions_.resize(info.joints.size(), 0.0);
    hw_velocities_.resize(info.joints.size(), 0.0);
    hw_prev_positions_.resize(info.joints.size(), 0.0); 
    
    port_ = info.hardware_parameters.at("device");
    baudrate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
    // ✅ Lấy wheel_radius và enc_counts_per_rev
    wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
    enc_counts_per_rev_ = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));
    dist_per_tick_ = 2.0 * M_PI * wheel_radius_ / static_cast<double>(enc_counts_per_rev_);

    RCLCPP_INFO(rclcpp::get_logger("STM32Hardware"),"Init: device=%s, baud=%d, wheel_radius=%.4f, enc_counts_per_rev=%d",port_.c_str(), baudrate_, wheel_radius_, enc_counts_per_rev_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32DiffDriveHardware::on_activate(const rclcpp_lifecycle::State &)
{
    try {
        serial_.open(port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    } catch (boost::system::system_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("STM32Hardware"), "Không thể mở serial %s: %s", port_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    std::fill(hw_prev_positions_.begin(), hw_prev_positions_.end(), 0.0);

    serial_read_buffer_.clear();

    RCLCPP_INFO(rclcpp::get_logger("STM32Hardware"), "STM32 hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (serial_.is_open()) {
        serial_.close();
    }
    RCLCPP_INFO(rclcpp::get_logger("STM32Hardware"), "STM32 hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================
// HÀM READ (ĐỌC) - FIX BLOCKING I/O
// ==========================================
hardware_interface::return_type STM32DiffDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Fix warning: Tắt cảnh báo về tham số 'time' không được sử dụng
    //(void)time; 
    
    if (!serial_.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("STM32"), "Serial port is not open in read().");
        return hardware_interface::return_type::ERROR;
    }

    // --- BƯỚC 1: ĐỌC DỮ LIỆU KHÔNG CHẶN (NON-BLOCKING) ---
    size_t bytes_available = 0;
    try {
        // SỬ DỤNG IOCTL HỖ TRỢ NON-BLOCKING
        bytes_available = get_bytes_available(serial_);
    } catch (boost::system::system_error & e) {
        RCLCPP_ERROR(rclcpp::get_logger("STM32"), "Serial available check failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    if (bytes_available > 0) {
        try {
            char temp_buffer[256];
            size_t bytes_to_read = std::min(bytes_available, sizeof(temp_buffer));
            // Đọc tất cả các byte có sẵn (NON-BLOCKING)
            size_t bytes_read = serial_.read_some(boost::asio::buffer(temp_buffer, bytes_to_read));
            serial_read_buffer_.append(temp_buffer, bytes_read); // Thêm vào buffer chờ xử lý
        } catch (boost::system::system_error & e) {
            RCLCPP_ERROR(rclcpp::get_logger("STM32"), "Serial read_some failed: %s", e.what());
        }
    }
    
    // --- BƯỚC 2: PHÂN TÍCH BUFFER CHO GÓI TIN ĐẦY ĐỦ ---
    size_t delimiter_pos = serial_read_buffer_.find('\n');
    if (delimiter_pos == std::string::npos) {
        // Gói tin chưa hoàn chỉnh, trả về OK ngay lập tức (NON-BLOCKING!)
        return hardware_interface::return_type::OK; 
    }
    
    std::string rx = serial_read_buffer_.substr(0, delimiter_pos);
    serial_read_buffer_.erase(0, delimiter_pos + 1); // Xóa gói tin đã xử lý

    // Debug log nhận dữ liệu
    RCLCPP_DEBUG(rclcpp::get_logger("STM32Hardware"), "Received data from STM32: '%s'", rx.c_str());

    // --- BƯỚC 3: PARSE VÀ CẬP NHẬT TRẠNG THÁI ---
    int tick_r = 0;
    int tick_l = 0;
    
    // Giao thức TX từ STM32: [TICK_R]r[TICK_L]l (Integer Ticks)
    if (sscanf(rx.c_str(), "%dr%dl", &tick_r, &tick_l) == 2) {
        RCLCPP_INFO(rclcpp::get_logger("STM32Hardware"),"RX encoder ticks: R=%d, L=%d", tick_r, tick_l);

        // Chuyển tick thành quãng đường [m]
        const double current_pos_l = static_cast<double>(tick_l) * dist_per_tick_;
        const double current_pos_r = static_cast<double>(tick_r) * dist_per_tick_;

         const double delta_time = period.seconds();
        
        if (delta_time > 0.0) {
            hw_velocities_[0] = (current_pos_l - hw_prev_positions_[0]) / delta_time; 
            hw_velocities_[1] = (current_pos_r - hw_prev_positions_[1]) / delta_time; 
        } else {
            hw_velocities_[0] = 0.0;
            hw_velocities_[1] = 0.0;
        }
        
        hw_prev_positions_[0] = current_pos_l;
        hw_prev_positions_[1] = current_pos_r;
        
        hw_positions_[0] = current_pos_l;
        hw_positions_[1] = current_pos_r;


        return hardware_interface::return_type::OK;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("STM32Hardware"),"Parse error RX: '%s'", rx.c_str());
    }
    return hardware_interface::return_type::OK;
}

// ==========================================
// HÀM WRITE (GHI) - Đồng bộ Giao thức TX
// ==========================================
hardware_interface::return_type STM32DiffDriveHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!serial_.is_open()) return hardware_interface::return_type::ERROR;

     float cmd_left_rad_per_s = hw_commands_[0];;  // Tốc độ góc cố định cho bánh trái (rad/s)
     float cmd_right_rad_per_s = hw_commands_[1]; // Tốc độ góc cố định cho bánh phải (rad/s)
    // float cmd_left_rad_per_s = 0;  // Tốc độ góc cố định cho bánh trái (rad/s)
    // float cmd_right_rad_per_s = 5; // Tốc độ góc cố định cho bánh phải (rad/s)
    
    // Tuỳ ý: giới hạn tốc độ để không điên
    constexpr float MAX_RAD_S = 50.0;
    constexpr float MIN_RAD_S = -50.0;

    // cmd_left_rad_per_s  = std::clamp(cmd_left_rad_per_s,  MIN_RAD_S, MAX_RAD_S);
    // cmd_right_rad_per_s = std::clamp(cmd_right_rad_per_s, MIN_RAD_S, MAX_RAD_S);

    RCLCPP_INFO(rclcpp::get_logger("STM32"), "Send cmd(rad/s): L=%.3f, R=%.3f", cmd_left_rad_per_s, cmd_right_rad_per_s);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << cmd_right_rad_per_s << " r " << cmd_left_rad_per_s << " l\n";
    
    const std::string tx = ss.str();
    RCLCPP_INFO(
        rclcpp::get_logger("STM32"),
        "TX to STM32: '%s'", tx.c_str());
    
    try {
        // boost::asio::write là blocking, nhưng vì TX thường nhanh hơn RX, ta chấp nhận
        boost::asio::write(serial_, boost::asio::buffer(ss.str()));
    } catch (boost::system::system_error & e) {
        RCLCPP_ERROR(rclcpp::get_logger("STM32"), "Serial write error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> STM32DiffDriveHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> STM32DiffDriveHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
}

} // namespace stm32_diff_drive_hardware

PLUGINLIB_EXPORT_CLASS(
    stm32_diff_drive_hardware::STM32DiffDriveHardware,
    hardware_interface::SystemInterface)