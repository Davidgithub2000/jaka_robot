#include <ros/ros.h>
#include <z_efg_ros/GripperCmd.h>
#include <z_efg_ros/GripperState.h>
#include <z_efg_ros/InterpretedState.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <thread>
#include <map>
#include <mutex>
#include <atomic>
#include <sys/select.h>
#include <errno.h>

class MultiGripperController {
private:
    int serial_fd_;
    std::string port_;
    int baud_rate_;
    bool running_;
    std::atomic<bool> command_running_;
    std::mutex serial_mutex_;
    std::thread state_thread_;

    struct GripperConfig {
        uint8_t id;
        float speed_open, speed_close;
        float current_open, current_close;
        float pos_open, pos_close;
        float current_position;
        float current_current;
        std::string interpreted_state;
    };

    std::map<uint8_t, GripperConfig> grippers_;
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher interpreted_pub_;

    // ---------- CRC16 ----------
    uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
        return crc;
    }

    // ---------- 串口打开（优化版） ----------
    int open_serial(const std::string& port, int baud) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            return -1;
        }

        speed_t speed;
        switch (baud) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            default: speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            return -1;
        }
        tcflush(fd, TCIOFLUSH);
        return fd;
    }

    // ---------- Modbus 发送（带重试） ----------
    bool send_modbus(uint8_t slave_id, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len, int retries = 1) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ < 0) return false;

        for (int attempt = 0; attempt <= retries; ++attempt) {
            if (attempt > 0) {
                tcflush(serial_fd_, TCIOFLUSH);
                usleep(50000);
            }
            tcflush(serial_fd_, TCIOFLUSH);
            if (write(serial_fd_, tx, tx_len) != (ssize_t)tx_len) continue;

            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(serial_fd_, &read_fds);
            timeout.tv_sec = 0;
            timeout.tv_usec = 500000;
            int ret = select(serial_fd_ + 1, &read_fds, NULL, NULL, &timeout);
            if (ret <= 0) {
                tcflush(serial_fd_, TCIOFLUSH);
                continue;
            }
            ssize_t n = read(serial_fd_, rx, rx_len);
            if (n < 4) {
                tcflush(serial_fd_, TCIOFLUSH);
                continue;
            }
            uint16_t crc = crc16(rx, n - 2);
            uint16_t rx_crc = rx[n-2] | (rx[n-1] << 8);
            if (crc != rx_crc) {
                tcflush(serial_fd_, TCIOFLUSH);
                continue;
            }
            tcflush(serial_fd_, TCIFLUSH);
            return true;
        }
        return false;
    }

    bool write_register(uint8_t slave_id, uint16_t addr, uint16_t value) {
        uint8_t tx[8], rx[8];
        tx[0] = slave_id;
        tx[1] = 0x06;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = (value >> 8) & 0xFF;
        tx[5] = value & 0xFF;
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        return send_modbus(slave_id, tx, 8, rx, 8);
    }

    bool write_float(uint8_t slave_id, uint16_t addr, float value) {
        union { float f; uint32_t i; } u;
        u.f = value;
        uint16_t high = (u.i >> 16) & 0xFFFF;
        uint16_t low = u.i & 0xFFFF;
        uint8_t tx[13], rx[8];
        tx[0] = slave_id;
        tx[1] = 0x10;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = 0x00;
        tx[5] = 0x02;
        tx[6] = 0x04;
        tx[7] = (high >> 8) & 0xFF;
        tx[8] = high & 0xFF;
        tx[9] = (low >> 8) & 0xFF;
        tx[10] = low & 0xFF;
        uint16_t crc = crc16(tx, 11);
        tx[11] = crc & 0xFF;
        tx[12] = (crc >> 8) & 0xFF;
        return send_modbus(slave_id, tx, 13, rx, 8);
    }

    uint16_t read_register(uint8_t slave_id, uint16_t addr) {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id;
        tx[1] = 0x03;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = 0x00;
        tx[5] = 0x01;
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        if (!send_modbus(slave_id, tx, 8, rx, 256)) return 0xFFFF;
        return (rx[3] << 8) | rx[4];
    }

    float read_position(uint8_t slave_id) {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id;
        tx[1] = 0x03;
        tx[2] = 0x00;
        tx[3] = 0x42;
        tx[4] = 0x00;
        tx[5] = 0x02;
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        if (!send_modbus(slave_id, tx, 8, rx, 256)) return std::nanf("");
        union { float f; uint32_t i; } u;
        u.i = (rx[3] << 24) | (rx[4] << 16) | (rx[5] << 8) | rx[6];
        return u.f;
    }

    void enable_motor(uint8_t slave_id) {
        write_register(slave_id, 0x0016, 0x0001);
        ros::Duration(0.2).sleep();
    }

    bool init_gripper(uint8_t slave_id) {
        bool ok_reset = write_register(slave_id, 0x0082, 0x0000);
        bool ok_init = write_register(slave_id, 0x0000, 0x0001);
        ros::Duration(3.0).sleep();
        bool ok_enable = write_register(slave_id, 0x0016, 0x0001);

        if (ok_reset && ok_init && ok_enable) {
            ROS_INFO("Gripper %d initialized and enabled", slave_id);
            return true;
        }

        ROS_ERROR(
            "Gripper %d initialization failed: reset=%d, init=%d, enable=%d",
            slave_id,
            ok_reset,
            ok_init,
            ok_enable
        );
        return false;
    }

    void move_to(uint8_t slave_id, float position, float speed, float current) {
        while (command_running_.load()) {
            usleep(10000);
        }
        command_running_ = true;

        bool ok_speed = write_float(slave_id, 0x0004, speed);
        bool ok_current = write_float(slave_id, 0x0006, current);
        bool ok_position = write_float(slave_id, 0x0002, position);

        if (ok_speed && ok_current && ok_position) {
            ROS_INFO(
                "Gripper %d moving to %.1f mm (speed=%.1f, current=%.2f)",
                slave_id,
                position,
                speed,
                current
            );
        } else {
            ROS_ERROR(
                "Gripper %d move command write failed: speed=%d, current=%d, position=%d",
                slave_id,
                ok_speed,
                ok_current,
                ok_position
            );
        }
        command_running_ = false;
    }

    // ---------- 5Hz 固定轮询 ----------
    void update_all_states() {
        ros::Rate rate(5);

        while (running_ && ros::ok()) {
            for (auto& pair : grippers_) {
                uint8_t id = pair.first;
                GripperConfig& g = pair.second;

                float pos = read_position(id);
                if (std::isnan(pos)) {
                    ROS_WARN_THROTTLE(1, "Failed to read position for gripper %d", id);
                } else {
                    g.current_position = pos;
                    std::string interpreted;
                    if (pos < 5.0) interpreted = "close";
                    else if (std::abs(pos - 20.0) < 0.5) interpreted = "ready";
                    else if (pos > 45.0) interpreted = "open";
                    else interpreted = "moving";
                    g.interpreted_state = interpreted;
                }

                // 发布 interpreted_state
                if (!g.interpreted_state.empty()) {
                    z_efg_ros::InterpretedState interpreted_msg;
                    interpreted_msg.id = id;
                    interpreted_msg.interpreted_state = g.interpreted_state;
                    interpreted_pub_.publish(interpreted_msg);
                }

                // 发布 state
                z_efg_ros::GripperState state_msg;
                state_msg.id = id;
                state_msg.state = g.interpreted_state.empty() ? "unknown" : g.interpreted_state;
                state_msg.position = g.current_position;
                state_msg.current = g.current_current;
                state_pub_.publish(state_msg);
            }

            rate.sleep();
        }
    }

    // ---------- 话题回调 ----------
    void cmd_callback(const z_efg_ros::GripperCmd::ConstPtr& msg) {
        ROS_INFO("Received command: id=%d, grip=%d", msg->id, msg->grip);
        uint8_t id = msg->id;
        if (grippers_.find(id) == grippers_.end()) {
            ROS_WARN("Unknown gripper ID: %d (only 1 and 2 are available)", id);
            return;
        }
        auto& g = grippers_[id];
        if (msg->grip) {
            move_to(id, g.pos_close, g.speed_close, g.current_close);
        } else {
            move_to(id, g.pos_open, g.speed_open, g.current_open);
        }
    }

    // ---------- 退出时张开 ----------
    void open_all_grippers() {
        if (serial_fd_ < 0) return;
        ROS_INFO("Opening all grippers on shutdown...");
        for (auto& pair : grippers_) {
            uint8_t id = pair.first;
            auto& g = pair.second;
            write_float(id, 0x0004, g.speed_open);
            write_float(id, 0x0006, g.current_open);
            write_float(id, 0x0002, g.pos_open);
            ROS_INFO("Gripper %d opened", id);
        }
        usleep(500000);
    }

public:
    MultiGripperController() : serial_fd_(-1), running_(true), command_running_(false) {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("port", port_, "/dev/grip_485");
        pnh.param<int>("baud", baud_rate_, 115200);

        float speed_open_param, speed_close_param;
        float current_open_param, current_close_param;
        float pos_open_param, pos_close_param;

        pnh.param<float>("speed_open", speed_open_param, 80.0);
        pnh.param<float>("speed_close", speed_close_param, 80.0);
        pnh.param<float>("current_open", current_open_param, 0.3);
        pnh.param<float>("current_close", current_close_param, 0.5);
        pnh.param<float>("pos_open", pos_open_param, 50.0);
        pnh.param<float>("pos_close", pos_close_param, 0.0);

        // ========== 修改：只创建 ID=1 和 ID=2 ==========
        for (uint8_t id : {1, 2}) {
            GripperConfig g;
            g.id = id;
            g.speed_open = speed_open_param;
            g.speed_close = speed_close_param;
            g.current_open = current_open_param;
            g.current_close = current_close_param;
            g.pos_open = pos_open_param;
            g.pos_close = pos_close_param;
            g.current_position = pos_open_param;
            g.current_current = 0.0;
            g.interpreted_state = "open";
            grippers_[id] = g;
        }
    }

    ~MultiGripperController() {
        running_ = false;
        if (state_thread_.joinable()) state_thread_.join();
        open_all_grippers();
        if (serial_fd_ >= 0) close(serial_fd_);
    }

    bool init() {
        serial_fd_ = open_serial(port_, baud_rate_);
        if (serial_fd_ < 0) {
            ROS_ERROR("Failed to open %s", port_.c_str());
            return false;
        }
        ROS_INFO("Serial port opened: %s", port_.c_str());

        for (auto& pair : grippers_) {
            init_gripper(pair.first);
        }

        interpreted_pub_ = nh_.advertise<z_efg_ros::InterpretedState>("/multi_gripper/interpreted_state", 10);
        state_pub_ = nh_.advertise<z_efg_ros::GripperState>("/multi_gripper/state", 10);
        cmd_sub_ = nh_.subscribe("/multi_gripper/cmd", 10, &MultiGripperController::cmd_callback, this);
        ROS_INFO("Subscribers created, ready to receive commands.");

        state_thread_ = std::thread(&MultiGripperController::update_all_states, this);

        ROS_INFO("Auto-opening all grippers on power-up...");
        for (auto& pair : grippers_) {
            uint8_t id = pair.first;
            auto& g = pair.second;
            move_to(id, g.pos_open, g.speed_open, g.current_open);
        }

        ROS_INFO("Multi-gripper ready (2 grippers, 5Hz polling, state published)");
        ROS_INFO("Available IDs: 1, 2");
        return true;
    }

    void spin() { ros::spin(); }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_gripper_node");
    MultiGripperController controller;
    if (!controller.init()) return -1;
    ROS_INFO("Multi-gripper node started (2 grippers)");
    controller.spin();
    return 0;
}
