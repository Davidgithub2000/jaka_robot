#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <thread>
#include <memory>

class SingleGripper {
private:
    int serial_fd;
    std::string port_;
    int baud_rate_;
    uint8_t slave_id_;
    
    float speed_open_, speed_close_;
    float current_open_, current_close_;
    float pos_open_, pos_close_;
    
    float current_position_;
    std::string current_state_;
    bool running_;
    std::unique_ptr<std::thread> state_thread_;
    
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Publisher pos_pub_;
    ros::Subscriber grip_sub_;
    ros::ServiceServer open_srv_;
    ros::ServiceServer close_srv_;
    
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
            case 115200: speed = B115200; break;
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
    
    bool send_modbus(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
        if (serial_fd < 0) return false;
        
        write(serial_fd, tx, tx_len);
        usleep(100000);
        
        ssize_t n = read(serial_fd, rx, rx_len);
        if (n < 4) return false;
        
        uint16_t crc = crc16(rx, n - 2);
        uint16_t rx_crc = rx[n-2] | (rx[n-1] << 8);
        return crc == rx_crc;
    }
    
    bool write_register(uint16_t addr, uint16_t value) {
        uint8_t tx[8], rx[8];
        tx[0] = slave_id_;
        tx[1] = 0x06;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = (value >> 8) & 0xFF;
        tx[5] = value & 0xFF;
        
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        
        return send_modbus(tx, 8, rx, 8);
    }
    
    bool write_float(uint16_t addr, float value) {
        union { float f; uint32_t i; } u;
        u.f = value;
        
        uint16_t high = (u.i >> 16) & 0xFFFF;
        uint16_t low = u.i & 0xFFFF;
        
        uint8_t tx[13], rx[8];
        tx[0] = slave_id_;
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
        
        return send_modbus(tx, 13, rx, 8);
    }
    
    float read_float(uint16_t addr) {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id_;
        tx[1] = 0x03;
        tx[2] = (addr >> 8) & 0xFF;
        tx[3] = addr & 0xFF;
        tx[4] = 0x00;
        tx[5] = 0x02;
        
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        
        if (!send_modbus(tx, 8, rx, 256)) return 0.0f;
        
        union { float f; uint32_t i; } u;
        u.i = (rx[3] << 24) | (rx[4] << 16) | (rx[5] << 8) | rx[6];
        return u.f;
    }
    
    uint16_t read_status() {
        uint8_t tx[8], rx[256];
        tx[0] = slave_id_;
        tx[1] = 0x03;
        tx[2] = 0x00;
        tx[3] = 0x41;
        tx[4] = 0x00;
        tx[5] = 0x01;
        
        uint16_t crc = crc16(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;
        
        if (!send_modbus(tx, 8, rx, 256)) return 0xFFFF;
        
        return (rx[3] << 8) | rx[4];
    }
    
    void init_gripper() {
        ROS_INFO("Initializing gripper...");
        write_register(0x0082, 0x0000);
        write_register(0x0000, 0x0001);
        sleep(3);
        write_register(0x0016, 0x0001);
        ROS_INFO("Gripper initialized");
    }
    
    void move_to(float position, float speed, float current) {
        write_float(0x0004, speed);
        write_float(0x0006, current);
        write_float(0x0002, position);
        ROS_INFO("Moving to %.1f mm (speed=%.1f, current=%.2f)", position, speed, current);
    }
    
    void update_state() {
        ros::Rate rate(10);
        while (running_ && ros::ok()) {
            current_position_ = read_float(0x0042);
            uint16_t status = read_status();
            
            switch (status) {
                case 0: current_state_ = "open"; break;
                case 1: current_state_ = "moving"; break;
                case 2: current_state_ = "gripping"; break;
                case 3: current_state_ = "dropped"; break;
                default: current_state_ = "unknown"; break;
            }
            
            std_msgs::String state_msg;
            state_msg.data = current_state_;
            state_pub_.publish(state_msg);
            
            std_msgs::Float32 pos_msg;
            pos_msg.data = current_position_;
            pos_pub_.publish(pos_msg);
            
            rate.sleep();
        }
    }
    
public:
    SingleGripper() : serial_fd(-1), slave_id_(1), current_position_(50.0), current_state_("unknown"), running_(true) {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("port", port_, "/dev/grip_485");
        pnh.param<int>("baud", baud_rate_, 115200);
        pnh.param<int>("slave_id", (int&)slave_id_, 1);
        pnh.param<float>("speed_open", speed_open_, 80.0);
        pnh.param<float>("speed_close", speed_close_, 80.0);
        pnh.param<float>("current_open", current_open_, 0.3);
        pnh.param<float>("current_close", current_close_, 0.5);
        pnh.param<float>("pos_open", pos_open_, 50.0);
        pnh.param<float>("pos_close", pos_close_, 0.0);
    }
    
    ~SingleGripper() {
        running_ = false;
        if (state_thread_ && state_thread_->joinable()) {
            state_thread_->join();
        }
        if (serial_fd >= 0) close(serial_fd);
    }
    
    bool init() {
        serial_fd = open_serial(port_, baud_rate_);
        if (serial_fd < 0) {
            ROS_ERROR("Failed to open %s", port_.c_str());
            return false;
        }
        ROS_INFO("Serial port opened: %s", port_.c_str());
        
        init_gripper();
        
        state_pub_ = nh_.advertise<std_msgs::String>("/single_gripper/state", 10);
        pos_pub_ = nh_.advertise<std_msgs::Float32>("/single_gripper/position", 10);
        grip_sub_ = nh_.subscribe("/single_gripper/grip_cmd", 10, &SingleGripper::grip_callback, this);
        open_srv_ = nh_.advertiseService("/single_gripper/open", &SingleGripper::open_callback, this);
        close_srv_ = nh_.advertiseService("/single_gripper/close", &SingleGripper::close_callback, this);
        
        state_thread_.reset(new std::thread(&SingleGripper::update_state, this));
        
        ROS_INFO("Single gripper ready. Use: 1=close(grip), 0=open(release)");
        return true;
    }
    
    void grip_callback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            move_to(pos_close_, speed_close_, current_close_);
            ROS_INFO("Command: CLOSE (1)");
        } else {
            move_to(pos_open_, speed_open_, current_open_);
            ROS_INFO("Command: OPEN (0)");
        }
    }
    
    bool open_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        move_to(pos_open_, speed_open_, current_open_);
        res.success = true;
        res.message = "Gripper opened";
        return true;
    }
    
    bool close_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        move_to(pos_close_, speed_close_, current_close_);
        res.success = true;
        res.message = "Gripper closed";
        return true;
    }
    
    void spin() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_gripper_node");
    
    SingleGripper gripper;
    
    if (!gripper.init()) {
        ROS_ERROR("Failed to initialize gripper");
        return -1;
    }
    
    ROS_INFO("Single gripper node started");
    ROS_INFO("Control: 1=Close(grip), 0=Open(release)");
    gripper.spin();
    
    return 0;
}
