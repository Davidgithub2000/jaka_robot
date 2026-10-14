#include <ros/ros.h>
#include <z_efg_ros/GripperCmd.h>
#include <z_efg_ros/GripperState.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <thread>
#include <map>

class MultiGripperController {
private:
    int serial_fd_;
    std::string port_;
    int baud_rate_;
    bool running_;
    
    struct GripperConfig {
        uint8_t id;
        float speed_open, speed_close;
        float current_open, current_close;
        float pos_open, pos_close;
        float current_position;
        float current_current;
        std::string state;
    };
    
    std::map<uint8_t, GripperConfig> grippers_;
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber cmd_sub_;
    
    uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }
    
    int open_serial(const std::string& port, int baud) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd, &tty);
        
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
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_lflag = 0;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;
        
        tcsetattr(fd, TCSANOW, &tty);
        return fd;
    }
    
    bool send_modbus(uint8_t slave_id, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
        if (serial_fd_ < 0) return false;
        write(serial_fd_, tx, tx_len);
        usleep(100000);
        ssize_t n = read(serial_fd_, rx, rx_len);
        if (n < 4) return false;
        uint16_t crc = crc16(rx, n - 2);
        uint16_t rx_crc = rx[n-2] | (rx[n-1] << 8);
        return crc == rx_crc;
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
    
    float read_float(uint8_t slave_id, uint16_t addr) {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id;
        tx[1] = 0x03;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = 0x00;
        tx[5] = 0x02;
        
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        
        if (!send_modbus(slave_id, tx, 8, rx, 256)) return 0.0f;
        
        union { float f; uint32_t i; } u;
        u.i = (rx[3] << 24) | (rx[4] << 16) | (rx[5] << 8) | rx[6];
        return u.f;
    }
    
    uint16_t read_status(uint8_t slave_id) {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id;
        tx[1] = 0x03;
        tx[2] = 0x00;
        tx[3] = 0x41;
        tx[4] = 0x00;
        tx[5] = 0x01;
        
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        
        if (!send_modbus(slave_id, tx, 8, rx, 256)) return 0xFFFF;
        
        return (rx[3] << 8) | rx[4];
    }
    
    void init_gripper(uint8_t slave_id) {
        ROS_INFO("Initializing gripper ID=%d...", slave_id);
        write_register(slave_id, 0x0082, 0x0000);
        write_register(slave_id, 0x0000, 0x0001);
        usleep(100000);
        write_register(slave_id, 0x0016, 0x0001);
        ROS_INFO("Gripper ID=%d ready", slave_id);
    }
    
    void move_to(uint8_t slave_id, float position, float speed, float current) {
        write_float(slave_id, 0x0004, speed);
        write_float(slave_id, 0x0006, current);
        write_float(slave_id, 0x0002, position);
        ROS_INFO("Gripper %d -> %.1f mm", slave_id, position);
    }
    
    void update_all_states() {
        ros::Rate rate(20);
        while (running_ && ros::ok()) {
            for (auto& pair : grippers_) {
                uint8_t id = pair.first;
                GripperConfig& g = pair.second;
                
                g.current_position = read_float(id, 0x0042);
                g.current_current = read_float(id, 0x0046);
                uint16_t status = read_status(id);
                
                switch (status) {
                    case 0: g.state = "open"; break;
                    case 1: g.state = "moving"; break;
                    case 2: g.state = "gripping"; break;
                    case 3: g.state = "dropped"; break;
                    default: g.state = "unknown"; break;
                }
                
                z_efg_ros::GripperState state_msg;
                state_msg.id = id;
                state_msg.state = g.state;
                state_msg.position = g.current_position;
                state_msg.current = g.current_current;
                state_pub_.publish(state_msg);
            }
            rate.sleep();
        }
    }
    
    void cmd_callback(const z_efg_ros::GripperCmd::ConstPtr& msg) {
        uint8_t id = msg->id;
        if (grippers_.find(id) == grippers_.end()) {
            ROS_WARN("Unknown gripper ID: %d", id);
            return;
        }
        
        auto& g = grippers_[id];
        if (msg->grip) {
            move_to(id, g.pos_close, g.speed_close, g.current_close);
            ROS_INFO("Gripper %d: CLOSE (1)", id);
        } else {
            move_to(id, g.pos_open, g.speed_open, g.current_open);
            ROS_INFO("Gripper %d: OPEN (0)", id);
        }
    }
    
public:
    MultiGripperController() : serial_fd_(-1), running_(true) {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("port", port_, "/dev/grip_485");
        pnh.param<int>("baud", baud_rate_, 115200);
        
        for (uint8_t id : {1, 2, 3}) {
            GripperConfig g;
            g.id = id;
            g.speed_open = 80.0;
            g.speed_close = 80.0;
            g.current_open = 0.3;
            g.current_close = 0.5;
            g.pos_open = 50.0;
            g.pos_close = 0.0;
            g.current_position = 50.0;
            g.current_current = 0.0;
            g.state = "unknown";
            grippers_[id] = g;
        }
    }
    
    ~MultiGripperController() {
        running_ = false;
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
        
        state_pub_ = nh_.advertise<z_efg_ros::GripperState>("/multi_gripper/state", 10);
        cmd_sub_ = nh_.subscribe("/multi_gripper/cmd", 10, &MultiGripperController::cmd_callback, this);
        
        new std::thread(&MultiGripperController::update_all_states, this);
        
        ROS_INFO("Multi-gripper ready. Use: 1=close(grip), 0=open(release)");
        return true;
    }
    
    void spin() { ros::spin(); }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_gripper_node");
    
    MultiGripperController controller;
    if (!controller.init()) return -1;
    
    ROS_INFO("Multi-gripper node started");
    ROS_INFO("Control format: id=X, grip=1(close) or 0(open)");
    controller.spin();
    return 0;
}
