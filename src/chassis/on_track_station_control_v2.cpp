#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

/*
 * on_track_station_control_v2.cpp
 *
 * 双机械臂番茄采摘底盘主控节点。
 *
 * 联动流程：
 * 1. 底盘在采摘点停车；
 * 2. 发布 /dual_picker/station_start=true，通知视觉和协调器开始新点位；
 * 3. 发布 /vision_enable=true，并同步 /vision_enable_left、/vision_enable_right；
 * 4. 视觉按左右相机结果向 /left_arm/custom_arm_data 和 /right_arm/custom_arm_data 发布目标；
 * 5. 双臂 wrapper 完成每个目标后发布 target_result；
 * 6. dual_pick_coordinator 汇总左右臂结果和 vision_done 后发布 /dual_picker/station_done；
 * 7. 底盘收到 station_done 后关闭视觉，等待机械臂安全收回，再匀速前进到下一个点。
 *
 * 安全保护：
 * - 移动过程中始终关闭视觉目标发布；
 * - station_done 只在 WAIT_ARM_DONE 状态接收，避免上一站锁存消息误触发；
 * - 通过 max_track_distance_m 限制轨道累计行驶距离，默认 50 m。
 */

class BaseStationController
{
public:
    BaseStationController()
        : nh_(),
          pnh_("~"),
          state_(INIT_WAIT),
          station_index_(0),
          station_done_received_(false),
          picking_active_(false),
          emergency_stop_(false),
          track_end_detected_(false),
          track_end_qr_detected_(false),
          track_end_pending_after_pick_(false),
          reverse_mode_active_(false),
          station_has_target_(false),
          empty_station_skip_(false),
          move_direction_(1.0),
          reverse_mode_station_count_(0),
          track_distance_m_(0.0),
          move_start_track_distance_m_(0.0),
          last_vision_enable_valid_(false),
          last_vision_enable_(false),
          last_station_start_valid_(false),
          last_station_start_(false)
    {
        loadParams();

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 20);

        // latch=true: 后启动的视觉节点或协调器也能收到当前门控状态。
        vision_enable_pub_ = nh_.advertise<std_msgs::Bool>(vision_enable_topic_, 5, true);
        station_start_pub_ = nh_.advertise<std_msgs::Bool>(station_start_topic_, 5, true);
        state_pub_ = nh_.advertise<std_msgs::String>(base_state_topic_, 10, true);

        station_done_sub_ = nh_.subscribe(
            station_done_topic_, 10, &BaseStationController::stationDoneCallback, this);
        // 兼容旧流程的视觉直通换站信号；默认不直接驱动底盘换站。
        vision_go_next_sub_ = nh_.subscribe(
            vision_go_next_topic_, 10, &BaseStationController::visionGoNextCallback, this);

        emergency_stop_sub_ = nh_.subscribe(
            emergency_stop_topic_, 5, &BaseStationController::emergencyStopCallback, this);

        track_end_detected_sub_ = nh_.subscribe(
            track_end_detected_topic_, 5, &BaseStationController::trackEndDetectedCallback, this);
        // 二维码识别成功后发这个话题，底盘据此进入倒退采摘模式。
        track_end_qr_detected_sub_ = nh_.subscribe(
            track_end_qr_detected_topic_, 5, &BaseStationController::trackEndQrDetectedCallback, this);

        left_target_sub_ = nh_.subscribe(
            left_target_topic_, 20, &BaseStationController::targetCallback, this);
        right_target_sub_ = nh_.subscribe(
            right_target_topic_, 20, &BaseStationController::targetCallback, this);

        manual_start_sub_ = nh_.subscribe(
            manual_start_topic_, 5, &BaseStationController::manualStartCallback, this);

        // 启动时先进入绝对安全状态。
        stopBase();
        setVisionEnabled(false, true);
        setStationStart(false, true);
        ros::param::set(pick_task_param_, 0);

        enterState(INIT_WAIT);

        ROS_INFO("on_track_station_control_v2 started.");
        ROS_INFO("Initial station auto start: %s", auto_start_initial_station_ ? "true" : "false");
    }

    void spin()
    {
        ros::Rate rate(control_rate_hz_);

        while (ros::ok())
        {
            ros::spinOnce();
            update();
            publishStateIfNeeded();
            rate.sleep();
        }

        // 节点退出时保持安全状态。
        stopBase();
        setVisionEnabled(false, true);
        setStationStart(false, true);
        ros::param::set(pick_task_param_, 0);
    }

private:
    enum TaskState
    {
        INIT_WAIT = 0,
        WAIT_ARM_DONE = 1,
        ARM_SAFE_DELAY = 2,
        MOVING_TO_NEXT = 3,
        STOP_STABILIZE = 4,
        FINISHED = 5,
        EMERGENCY_STOP = 6,
        // TRACK_END_QR_WAIT：轨道到头后停车等待二维码。
        TRACK_END_QR_WAIT = 7,
        // REVERSING_FROM_TRACK_END：二维码识别后的第一次后退。
        REVERSING_FROM_TRACK_END = 8
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher cmd_pub_;
    ros::Publisher vision_enable_pub_;
    ros::Publisher station_start_pub_;
    ros::Publisher state_pub_;

    ros::Subscriber station_done_sub_;
    ros::Subscriber vision_go_next_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber track_end_detected_sub_;
    ros::Subscriber track_end_qr_detected_sub_;
    ros::Subscriber left_target_sub_;
    ros::Subscriber right_target_sub_;
    ros::Subscriber manual_start_sub_;

    TaskState state_;
    ros::Time state_start_time_;
    ros::Time last_state_pub_time_;

    int station_index_;
    bool station_done_received_;
    bool picking_active_;
    bool emergency_stop_;
    bool track_end_detected_;
    // 二维码相机已识别到有效二维码。
    bool track_end_qr_detected_;
    // Track-end signal received while arms are still picking; handle it after station_done.
    bool track_end_pending_after_pick_;
    // 扫码后进入倒退采摘模式，后续换工位都变成后退。
    bool reverse_mode_active_;
    bool station_has_target_;
    bool empty_station_skip_;

    // 状态发布用：1 表示前进，-1 表示后退。
    double move_direction_;
    // 进入倒退模式后已经后退经过的工位数。
    int reverse_mode_station_count_;

    double track_distance_m_;
    double move_start_track_distance_m_;

    bool last_vision_enable_valid_;
    bool last_vision_enable_;
    bool last_station_start_valid_;
    bool last_station_start_;

    // 话题名。
    std::string cmd_vel_topic_;
    std::string vision_enable_topic_;
    std::string station_start_topic_;
    std::string station_done_topic_;
    std::string vision_go_next_topic_;
    std::string base_state_topic_;
    std::string emergency_stop_topic_;
    std::string track_end_detected_topic_;
    std::string track_end_qr_detected_topic_;
    std::string manual_start_topic_;
    std::string left_target_topic_;
    std::string right_target_topic_;

    // 参数名。
    std::string pick_task_param_;
    std::string vision_enable_param_;
    std::string vision_enable_left_param_;
    std::string vision_enable_right_param_;

    // 运动和流程参数。
    double move_speed_;
    double move_duration_;
    double max_track_distance_m_;
    double arm_safe_delay_;
    double stop_stable_delay_;
    double startup_delay_;
    double track_end_reverse_speed_;
    double track_end_reverse_duration_;
    double track_end_qr_wait_timeout_;
    double control_rate_hz_;
    double state_publish_period_;
    double empty_station_timeout_;
    int max_station_count_;
    int reverse_mode_max_stations_;
    bool auto_start_initial_station_;
    bool enable_empty_station_timeout_;
    bool enable_track_end_qr_reverse_;
    bool allow_vision_go_next_direct_;

    void loadParams()
    {
        pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        pnh_.param<std::string>("vision_enable_topic", vision_enable_topic_, std::string("/vision_enable"));
        pnh_.param<std::string>("station_start_topic", station_start_topic_, std::string("/dual_picker/station_start"));
        pnh_.param<std::string>("station_done_topic", station_done_topic_, std::string("/dual_picker/station_done"));
        pnh_.param<std::string>("vision_go_next_topic", vision_go_next_topic_, std::string("/vision_master/go_next_station"));
        pnh_.param<std::string>("base_state_topic", base_state_topic_, std::string("/base/task_state"));
        pnh_.param<std::string>("emergency_stop_topic", emergency_stop_topic_, std::string("/base/emergency_stop"));
        pnh_.param<std::string>("track_end_detected_topic", track_end_detected_topic_, std::string("/base/track_end_detected"));
        pnh_.param<std::string>("track_end_qr_detected_topic", track_end_qr_detected_topic_, std::string("/base/track_end_qr_detected"));
        pnh_.param<std::string>("manual_start_topic", manual_start_topic_, std::string("/base/manual_start_station"));
        pnh_.param<std::string>("left_target_topic", left_target_topic_, std::string("/left_arm/custom_arm_data"));
        pnh_.param<std::string>("right_target_topic", right_target_topic_, std::string("/right_arm/custom_arm_data"));

        pnh_.param<std::string>("pick_task_param", pick_task_param_, std::string("/pick_task"));
        pnh_.param<std::string>("vision_enable_param", vision_enable_param_, std::string("/vision_enable"));
        pnh_.param<std::string>("vision_enable_left_param", vision_enable_left_param_, std::string("/vision_enable_left"));
        pnh_.param<std::string>("vision_enable_right_param", vision_enable_right_param_, std::string("/vision_enable_right"));

        // 底盘运动参数：move_speed 是速度，move_duration 是时间，二者相乘约等于每次移动距离。
        // track_end_reverse_* 用于到头扫码成功后的第一次后退。
        pnh_.param("move_speed", move_speed_, 0.17);
        pnh_.param("move_duration", move_duration_, 7.0);
        pnh_.param("max_track_distance_m", max_track_distance_m_, 50.0);
        pnh_.param("arm_safe_delay", arm_safe_delay_, 5.0);
        pnh_.param("stop_stable_delay", stop_stable_delay_, 3.0);
        pnh_.param("startup_delay", startup_delay_, 2.0);
        pnh_.param("track_end_reverse_speed", track_end_reverse_speed_, 0.12);
        pnh_.param("track_end_reverse_duration", track_end_reverse_duration_, 2.0);
        pnh_.param("track_end_qr_wait_timeout", track_end_qr_wait_timeout_, 20.0);
        pnh_.param("control_rate_hz", control_rate_hz_, 20.0);
        pnh_.param("state_publish_period", state_publish_period_, 1.0);
        pnh_.param("enable_empty_station_timeout", enable_empty_station_timeout_, true);
        pnh_.param("empty_station_timeout", empty_station_timeout_, 10.0);
        pnh_.param("max_station_count", max_station_count_, 0);
        // 倒退采摘模式最多走多少个工位，0 表示不按工位数限制。
        pnh_.param("reverse_mode_max_stations", reverse_mode_max_stations_, 0);
        pnh_.param("auto_start_initial_station", auto_start_initial_station_, true);
        // true 表示到头后扫码并进入倒退采摘；false 表示到头直接结束。
        pnh_.param("enable_track_end_qr_reverse", enable_track_end_qr_reverse_, true);
        // 新流程中 vision_go_next 只作为视觉状态信号，底盘必须等 dual_pick_coordinator 的 station_done。
        pnh_.param("allow_vision_go_next_direct", allow_vision_go_next_direct_, false);

        control_rate_hz_ = std::max(1.0, control_rate_hz_);
        state_publish_period_ = std::max(0.2, state_publish_period_);
        if (enable_empty_station_timeout_ && empty_station_timeout_ < 1.0)
        {
            ROS_WARN("empty_station_timeout is too small. Use 1.0 second minimum.");
            empty_station_timeout_ = 1.0;
        }

        if (move_speed_ < 0.0)
        {
            ROS_WARN("move_speed is negative. Use absolute value.");
            move_speed_ = std::abs(move_speed_);
        }
        if (track_end_reverse_speed_ < 0.0)
        {
            ROS_WARN("track_end_reverse_speed is negative. Use absolute value.");
            track_end_reverse_speed_ = std::abs(track_end_reverse_speed_);
        }
        if (track_end_reverse_duration_ < 0.1)
        {
            ROS_WARN("track_end_reverse_duration is too small. Use 0.1 second minimum.");
            track_end_reverse_duration_ = 0.1;
        }
        if (track_end_qr_wait_timeout_ < 0.0)
        {
            ROS_WARN("track_end_qr_wait_timeout is negative. Disable QR wait timeout.");
            track_end_qr_wait_timeout_ = 0.0;
        }
        if (reverse_mode_max_stations_ < 0)
        {
            ROS_WARN("reverse_mode_max_stations is negative. Disable reverse station limit.");
            reverse_mode_max_stations_ = 0;
        }
        if (max_track_distance_m_ < 0.0)
        {
            ROS_WARN("max_track_distance_m is negative. Disable track distance limit.");
            max_track_distance_m_ = 0.0;
        }
    }

    void update()
    {
        if (emergency_stop_)
        {
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);

            if (state_ != EMERGENCY_STOP)
            {
                enterState(EMERGENCY_STOP);
            }
            return;
        }

        // 兼容旧流程：如果关闭扫码倒退功能，轨道到头就直接结束。
        if (track_end_detected_ && !enable_track_end_qr_reverse_ && !(state_ == WAIT_ARM_DONE && picking_active_))
        {
            station_done_received_ = false;
            picking_active_ = false;

            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);

            if (state_ != FINISHED)
            {
                enterState(FINISHED);
            }
            return;
        }

        switch (state_)
        {
        case INIT_WAIT:
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);

            if (auto_start_initial_station_ && elapsed() >= startup_delay_)
            {
                startPickStation();
            }
            break;

        case WAIT_ARM_DONE:
            // 当前点位采摘中：底盘停车，等待协调器发布 station_done。
            stopBase();

            if (station_done_received_)
            {
                finishPickStation();
                enterState(ARM_SAFE_DELAY);
            }
            else if (shouldSkipEmptyStation())
            {
                skipEmptyStation();
                enterState(ARM_SAFE_DELAY);
            }
            else
            {
                warnIfEmptyStationTimeoutBlocked();
            }
            break;

        case ARM_SAFE_DELAY:
            // 双臂完成后再等待一段时间，避免机械臂尚未完全复位时底盘移动。
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);

            if (elapsed() >= arm_safe_delay_)
            {
                // 倒退模式下只看倒退工位上限，不再受正向 max_station_count 限制。
                if (track_end_pending_after_pick_ || track_end_detected_)
                {
                    track_end_pending_after_pick_ = false;
                    ROS_WARN("Track end was detected while picking. Handle it after arm safe delay.");
                    enterState(enable_track_end_qr_reverse_ ? TRACK_END_QR_WAIT : FINISHED);
                }
                else if (reverse_mode_active_ && reverse_mode_max_stations_ > 0 &&
                    reverse_mode_station_count_ >= reverse_mode_max_stations_)
                {
                    ROS_WARN("Reverse station limit reached: %d.", reverse_mode_station_count_);
                    enterState(FINISHED);
                }
                else if (hasReachedTrackLimit())
                {
                    ROS_WARN("Track distance limit reached before next move: %.3f / %.3f m.",
                             track_distance_m_, max_track_distance_m_);
                    enterState(FINISHED);
                }
                else if (!reverse_mode_active_ && max_station_count_ > 0 && station_index_ >= max_station_count_ - 1)
                {
                    enterState(FINISHED);
                }
                else
                {
                    enterState(MOVING_TO_NEXT);
                }
            }
            break;

        case MOVING_TO_NEXT:
        {
            // Move to the next picking station. Direction switches to reverse after track-end QR.
            setVisionEnabled(false);
            setStationStart(false);

            // 根据 reverse_mode_active_ 自动决定前进还是后退。
            const double signed_speed = currentMoveSpeed();
            const double raw_estimated_track_distance = move_start_track_distance_m_ + signed_speed * elapsed();
            const double estimated_track_distance = clampTrackDistance(raw_estimated_track_distance);
            if (!reverse_mode_active_ && max_track_distance_m_ > 0.0 && estimated_track_distance >= max_track_distance_m_)
            {
                stopBase();
                track_distance_m_ = max_track_distance_m_;
                ROS_WARN("Track distance limit reached: %.3f / %.3f m. Finish task.",
                         track_distance_m_, max_track_distance_m_);
                enterState(FINISHED);
                break;
            }
            // 倒退模式下估算已回到轨道起点时结束，避免继续后退。
            if (reverse_mode_active_ && move_start_track_distance_m_ > 0.0 && raw_estimated_track_distance <= 0.0)
            {
                stopBase();
                track_distance_m_ = 0.0;
                ROS_WARN("Reverse mode reached track start estimate. Finish task.");
                enterState(FINISHED);
                break;
            }

            publishVel(signed_speed, 0.0);

            if (elapsed() >= move_duration_)
            {
                stopBase();
                track_distance_m_ = clampTrackDistance(move_start_track_distance_m_ + signed_speed * move_duration_);
                if (reverse_mode_active_)
                {
                    reverse_mode_station_count_++;
                    station_index_ = std::max(0, station_index_ - 1);
                }
                else
                {
                    station_index_++;
                }
                enterState(STOP_STABILIZE);
            }
            break;
        }

        case STOP_STABILIZE:
            // 到达下一个点后先停车稳定，再开始新采摘点。
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);

            if (elapsed() >= stop_stable_delay_)
            {
                if (hasReachedTrackLimit())
                {
                    ROS_WARN("Track distance limit reached at station boundary: %.3f / %.3f m.",
                             track_distance_m_, max_track_distance_m_);
                    enterState(FINISHED);
                }
                else
                {
                    startPickStation();
                }
            }
            break;

        // 到头后等待二维码，识别前保持停车并关闭视觉。
        case TRACK_END_QR_WAIT:
            station_done_received_ = false;
            picking_active_ = false;
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);

            if (track_end_qr_detected_)
            {
                // 二维码确认后进入倒退采摘模式。
                reverse_mode_active_ = true;
                move_direction_ = -1.0;
                reverse_mode_station_count_ = 0;
                ROS_WARN("Track-end QR accepted. Start reverse picking mode.");
                enterState(REVERSING_FROM_TRACK_END);
            }
            else if (track_end_qr_wait_timeout_ > 0.0 && elapsed() >= track_end_qr_wait_timeout_)
            {
                ROS_WARN("Track-end QR wait timeout %.1f s. Finish task.", track_end_qr_wait_timeout_);
                enterState(FINISHED);
            }
            break;

        // 扫码成功后的第一次后退，离开轨道尽头后再开启新一轮采摘。
        case REVERSING_FROM_TRACK_END:
        {
            station_done_received_ = false;
            picking_active_ = false;
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);

            const double reverse_speed = -std::abs(track_end_reverse_speed_);
            publishVel(reverse_speed, 0.0);

            if (elapsed() >= track_end_reverse_duration_)
            {
                stopBase();
                track_distance_m_ = clampTrackDistance(
                    move_start_track_distance_m_ - std::abs(track_end_reverse_speed_) * track_end_reverse_duration_);
                track_end_detected_ = false;
                track_end_qr_detected_ = false;
                track_end_pending_after_pick_ = false;
                reverse_mode_station_count_++;
                station_index_ = std::max(0, station_index_ - 1);
                ROS_INFO("Reverse move after QR done. station=%d distance=%.3f m.",
                         station_index_, track_distance_m_);
                enterState(STOP_STABILIZE);
            }
            break;
        }

        case FINISHED:
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);
            break;

        case EMERGENCY_STOP:
            stopBase();
            setVisionEnabled(false);
            setStationStart(false);
            ros::param::set(pick_task_param_, 0);
            break;
        }
    }

    void startPickStation()
    {
        // 开始新采摘点：停车、重置完成标记、打开 station_start 和视觉门控。
        stopBase();

        station_done_received_ = false;
        station_has_target_ = false;
        empty_station_skip_ = false;
        picking_active_ = true;

        setStationStart(true, true);
        ros::param::set(pick_task_param_, 1);
        setVisionEnabled(true, true);

        ROS_INFO("Station %d start: vision enabled, waiting for dual arms done.",
                 station_index_);

        enterState(WAIT_ARM_DONE);
    }

    void finishPickStation()
    {
        // 当前采摘点结束：关视觉、关 station_start，并进入机械臂安全等待阶段。
        picking_active_ = false;

        stopBase();
        setVisionEnabled(false, true);
        setStationStart(false, true);
        ros::param::set(pick_task_param_, 0);

        ROS_INFO("Station %d done: vision disabled, enter arm safe delay.",
                 station_index_);
    }

    void skipEmptyStation()
    {
        empty_station_skip_ = true;
        picking_active_ = false;

        stopBase();
        setVisionEnabled(false, true);
        setStationStart(false, true);
        ros::param::set(pick_task_param_, 0);

        ROS_WARN("Station %d skipped as empty: no valid target for %.1f seconds.",
                 station_index_, empty_station_timeout_);
    }

    void targetCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
    {
        if (state_ != WAIT_ARM_DONE || !picking_active_ || !last_vision_enable_ || !last_station_start_)
        {
            return;
        }

        if (msg->data.size() < 6)
        {
            ROS_WARN_THROTTLE(2.0, "Ignore target message with too few values: %d.",
                              static_cast<int>(msg->data.size()));
            return;
        }

        if (isEmptyTargetPlaceholder(msg))
        {
            ROS_INFO_THROTTLE(5.0, "Ignore empty target placeholder on arm target topic.");
            return;
        }

        if (msg->data.size() >= 8)
        {
            const float first = msg->data[0];
            const float second = msg->data[1];
            const bool first_is_robot_id = isKnownRobotId(first);
            const bool second_is_robot_id = isKnownRobotId(second);
            const bool first_is_existed_flag = isExistedFlag(first);
            const bool second_is_existed_flag = isExistedFlag(second);

            if ((first_is_robot_id && second_is_existed_flag) ||
                (first_is_existed_flag && second_is_robot_id))
            {
                const float existed = first_is_robot_id ? second : first;
                if (existed < 0.5f)
                {
                    return;
                }
            }
            else if (first_is_robot_id || second_is_robot_id)
            {
                ROS_WARN_THROTTLE(2.0, "Ignore target message with invalid robot/existed fields: %.3f %.3f.",
                                  first, second);
                return;
            }
        }

        if (!station_has_target_)
        {
            ROS_INFO("Station %d received first valid target. Empty-station timeout disabled for this station.",
                     station_index_);
        }
        station_has_target_ = true;
    }

    bool isKnownRobotId(float value) const
    {
        return std::fabs(value - 181.0f) < 0.01f ||
               std::fabs(value - 188.0f) < 0.01f ||
               std::fabs(value - 1.0f) < 0.01f ||
               std::fabs(value - 2.0f) < 0.01f;
    }

    bool isExistedFlag(float value) const
    {
        return std::fabs(value) < 0.01f || std::fabs(value - 1.0f) < 0.01f;
    }

    bool isEmptyTargetPlaceholder(const std_msgs::Float32MultiArrayConstPtr& msg) const
    {
        if (msg->data.size() < 6)
        {
            return false;
        }

        for (size_t index = 0; index < 6; ++index)
        {
            if (std::fabs(msg->data[index]) > 1e-6f)
            {
                return false;
            }
        }
        return true;
    }

    bool hasTargetPublishers() const
    {
        return left_target_sub_.getNumPublishers() > 0 && right_target_sub_.getNumPublishers() > 0;
    }

    bool shouldSkipEmptyStation() const
    {
        if (!enable_empty_station_timeout_ || empty_station_timeout_ <= 0.0)
        {
            return false;
        }
        if (state_ != WAIT_ARM_DONE || !picking_active_ || !last_vision_enable_ || !last_station_start_)
        {
            return false;
        }
        if (station_has_target_ || !hasTargetPublishers())
        {
            return false;
        }
        return elapsed() >= empty_station_timeout_;
    }

    void warnIfEmptyStationTimeoutBlocked()
    {
        if (!enable_empty_station_timeout_ || station_has_target_ || elapsed() < empty_station_timeout_)
        {
            return;
        }

        const unsigned int left_publishers = left_target_sub_.getNumPublishers();
        const unsigned int right_publishers = right_target_sub_.getNumPublishers();
        if (left_publishers > 0 && right_publishers > 0)
        {
            return;
        }

        ROS_WARN_THROTTLE(5.0,
                          "Empty-station timeout blocked because target publishers are missing: left=%u right=%u.",
                          left_publishers, right_publishers);
    }

    void stationDoneCallback(const std_msgs::BoolConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        // 只在等待采摘完成状态接收 station_done，避免上一站 latch=true 的完成消息误触发移动。
        if (state_ == WAIT_ARM_DONE && picking_active_)
        {
            station_done_received_ = true;
            ROS_INFO("Receive /dual_picker/station_done=true at station %d.",
                     station_index_);
        }
        else
        {
            ROS_WARN("Ignore station_done because base is not waiting for picking result.");
        }
    }

    // 视觉无目标信号只给协调器确认工位结束；默认不允许它绕过机械臂完成判断直接驱动底盘换站。
    void visionGoNextCallback(const std_msgs::BoolConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        if (!allow_vision_go_next_direct_)
        {
            ROS_WARN_THROTTLE(2.0,
                              "Ignore vision go-next on %s; waiting for %s from dual_pick_coordinator.",
                              vision_go_next_topic_.c_str(), station_done_topic_.c_str());
            return;
        }

        if (state_ == WAIT_ARM_DONE && picking_active_)
        {
            station_done_received_ = true;
            ROS_WARN("Legacy vision go-next accepted on %s at station %d.",
                     vision_go_next_topic_.c_str(), station_index_);
        }
        else
        {
            ROS_WARN("Ignore vision go-next command because base is not waiting for picking result.");
        }
    }
    void emergencyStopCallback(const std_msgs::BoolConstPtr& msg)
    {
        emergency_stop_ = msg->data;

        if (emergency_stop_)
        {
            ROS_ERROR("Emergency stop received. Base stopped and vision disabled.");
            stopBase();
            setVisionEnabled(false, true);
            setStationStart(false, true);
            ros::param::set(pick_task_param_, 0);
            enterState(EMERGENCY_STOP);
        }
        else
        {
            if (track_end_detected_)
            {
                if (enable_track_end_qr_reverse_)
                {
                    ROS_WARN("Emergency stop released at track end. Return to QR wait.");
                    enterState(TRACK_END_QR_WAIT);
                }
                else
                {
                    ROS_WARN("Emergency stop released, but track end is already detected. Stay FINISHED.");
                    enterState(FINISHED);
                }
                return;
            }

            ROS_WARN("Emergency stop released. Return to INIT_WAIT.");
            enterState(INIT_WAIT);
        }
    }

    // 轨道到头：停车关视觉，启用扫码倒退时进入等待二维码。
    void trackEndDetectedCallback(const std_msgs::BoolConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        // 倒退采摘模式下忽略轨道尽头残留信号，避免重复进入扫码等待。
        if (reverse_mode_active_)
        {
            ROS_WARN_THROTTLE(5.0, "Ignore track-end signal during reverse picking mode.");
            return;
        }

        if (track_end_detected_)
        {
            return;
        }

        if (state_ == MOVING_TO_NEXT || state_ == REVERSING_FROM_TRACK_END)
        {
            track_distance_m_ = currentTrackDistanceEstimate();
        }

        track_end_detected_ = true;
        track_end_qr_detected_ = false;
        if (state_ == WAIT_ARM_DONE && picking_active_)
        {
            track_end_pending_after_pick_ = true;
            stopBase();
            ROS_WARN("Track end detected while arms are picking. Keep picking active and defer QR/reverse until station_done.");
            return;
        }

        track_end_pending_after_pick_ = false;
        station_done_received_ = false;
        picking_active_ = false;

        if (enable_track_end_qr_reverse_)
        {
            ROS_WARN("Track end detected on %s. Wait for QR before reverse mode.",
                     track_end_detected_topic_.c_str());
        }
        else
        {
            ROS_WARN("Track end detected on %s. Finish base task.",
                     track_end_detected_topic_.c_str());
        }

        stopBase();
        setVisionEnabled(false, true);
        setStationStart(false, true);
        ros::param::set(pick_task_param_, 0);

        if (!emergency_stop_)
        {
            enterState(enable_track_end_qr_reverse_ ? TRACK_END_QR_WAIT : FINISHED);
        }
    }

    // 只在等待二维码状态接受 true，防止旧消息误触发。
    void trackEndQrDetectedCallback(const std_msgs::BoolConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        if (!enable_track_end_qr_reverse_)
        {
            ROS_WARN("Ignore track-end QR because QR reverse mode is disabled.");
            return;
        }
        if (state_ == WAIT_ARM_DONE && track_end_pending_after_pick_ && track_end_detected_)
        {
            track_end_qr_detected_ = true;
            ROS_WARN("Receive track-end QR while arms are picking. Defer reverse until station_done.");
            return;
        }

        if (state_ != TRACK_END_QR_WAIT || !track_end_detected_)
        {
            ROS_WARN("Ignore track-end QR because base is not waiting for QR.");
            return;
        }

        track_end_qr_detected_ = true;
        ROS_WARN("Receive track-end QR on %s.", track_end_qr_detected_topic_.c_str());
    }

    void manualStartCallback(const std_msgs::BoolConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        if (emergency_stop_)
        {
            ROS_WARN("Manual start ignored because emergency stop is active.");
            return;
        }

        if (track_end_detected_)
        {
            ROS_WARN("Manual start ignored because track end is already detected.");
            return;
        }

        if (state_ == INIT_WAIT || state_ == STOP_STABILIZE)
        {
            ROS_INFO("Manual start station command received.");
            startPickStation();
        }
        else
        {
            ROS_WARN("Manual start ignored in state %s.", stateName(state_).c_str());
        }
    }

    void setVisionEnabled(bool enabled, bool force = false)
    {
        ros::param::set(vision_enable_param_, enabled);
        ros::param::set(vision_enable_left_param_, enabled);
        ros::param::set(vision_enable_right_param_, enabled);

        if (force || !last_vision_enable_valid_ || last_vision_enable_ != enabled)
        {
            publishBool(vision_enable_pub_, enabled);
            last_vision_enable_ = enabled;
            last_vision_enable_valid_ = true;
            ROS_INFO("vision_enable=%s", enabled ? "true" : "false");
        }
    }

    void setStationStart(bool started, bool force = false)
    {
        if (force || !last_station_start_valid_ || last_station_start_ != started)
        {
            publishBool(station_start_pub_, started);
            last_station_start_ = started;
            last_station_start_valid_ = true;
            ROS_INFO("station_start=%s", started ? "true" : "false");
        }
    }

    void publishVel(double linear_x, double angular_z)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = angular_z;
        cmd_pub_.publish(cmd);
    }

    void stopBase()
    {
        publishVel(0.0, 0.0);
    }

    void publishBool(ros::Publisher& pub, bool value)
    {
        std_msgs::Bool msg;
        msg.data = value;
        pub.publish(msg);
    }

    double currentMoveSpeed() const
    {
        return (reverse_mode_active_ ? -1.0 : 1.0) * std::abs(move_speed_);
    }

    double clampTrackDistance(double value) const
    {
        value = std::max(0.0, value);
        if (max_track_distance_m_ > 0.0)
        {
            value = std::min(value, max_track_distance_m_);
        }
        return value;
    }

    bool hasReachedTrackLimit() const
    {
        return !reverse_mode_active_ && max_track_distance_m_ > 0.0 && track_distance_m_ >= max_track_distance_m_;
    }

    double currentTrackDistanceEstimate() const
    {
        if (state_ == MOVING_TO_NEXT)
        {
            return clampTrackDistance(move_start_track_distance_m_ + currentMoveSpeed() * elapsed());
        }
        if (state_ == REVERSING_FROM_TRACK_END)
        {
            return clampTrackDistance(move_start_track_distance_m_ - std::abs(track_end_reverse_speed_) * elapsed());
        }
        return track_distance_m_;
    }

    void enterState(TaskState new_state)
    {
        if (new_state == MOVING_TO_NEXT)
        {
            move_start_track_distance_m_ = track_distance_m_;
            move_direction_ = reverse_mode_active_ ? -1.0 : 1.0;
        }
        else if (new_state == REVERSING_FROM_TRACK_END)
        {
            move_start_track_distance_m_ = track_distance_m_;
            move_direction_ = -1.0;
        }

        state_ = new_state;
        state_start_time_ = ros::Time::now();
        publishState(true);
    }

    double elapsed() const
    {
        return (ros::Time::now() - state_start_time_).toSec();
    }

    std::string stateName(TaskState state) const
    {
        switch (state)
        {
        case INIT_WAIT:
            return "INIT_WAIT";
        case WAIT_ARM_DONE:
            return "WAIT_ARM_DONE";
        case ARM_SAFE_DELAY:
            return "ARM_SAFE_DELAY";
        case MOVING_TO_NEXT:
            return "MOVING_TO_NEXT";
        case STOP_STABILIZE:
            return "STOP_STABILIZE";
        case FINISHED:
            return "FINISHED";
        case EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case TRACK_END_QR_WAIT:
            return "TRACK_END_QR_WAIT";
        case REVERSING_FROM_TRACK_END:
            return "REVERSING_FROM_TRACK_END";
        default:
            return "UNKNOWN";
        }
    }

    void publishStateIfNeeded()
    {
        if ((ros::Time::now() - last_state_pub_time_).toSec() >= state_publish_period_)
        {
            publishState(false);
        }
    }

    void publishState(bool force)
    {
        if (!force && (ros::Time::now() - last_state_pub_time_).toSec() < state_publish_period_)
        {
            return;
        }

        std_msgs::String msg;
        std::stringstream ss;
        ss << "{"
           << "\"state\":\"" << stateName(state_) << "\","
           << "\"station_index\":" << station_index_ << ","
           << "\"vision_enable\":" << (last_vision_enable_ ? "true" : "false") << ","
           << "\"station_start\":" << (last_station_start_ ? "true" : "false") << ","
           << "\"picking_active\":" << (picking_active_ ? "true" : "false") << ","
           << "\"emergency_stop\":" << (emergency_stop_ ? "true" : "false") << ","
           << "\"track_end_detected\":" << (track_end_detected_ ? "true" : "false") << ","
           << "\"track_end_qr_detected\":" << (track_end_qr_detected_ ? "true" : "false") << ","
           << "\"track_end_pending_after_pick\":" << (track_end_pending_after_pick_ ? "true" : "false") << ","
           << "\"reverse_mode_active\":" << (reverse_mode_active_ ? "true" : "false") << ","
           << "\"move_direction\":" << move_direction_ << ","
           << "\"reverse_mode_station_count\":" << reverse_mode_station_count_ << ","
           << "\"reverse_mode_max_stations\":" << reverse_mode_max_stations_ << ","
           << "\"enable_empty_station_timeout\":" << (enable_empty_station_timeout_ ? "true" : "false") << ","
           << "\"empty_station_timeout\":" << empty_station_timeout_ << ","
           << "\"station_has_target\":" << (station_has_target_ ? "true" : "false") << ","
           << "\"empty_station_skip\":" << (empty_station_skip_ ? "true" : "false") << ","
           << "\"target_publishers_ready\":" << (hasTargetPublishers() ? "true" : "false") << ","
           << "\"move_speed\":" << move_speed_ << ","
           << "\"move_duration\":" << move_duration_ << ","
           << "\"track_distance_m\":" << currentTrackDistanceEstimate() << ","
           << "\"max_track_distance_m\":" << max_track_distance_m_ << ","
           << "\"elapsed\":" << elapsed()
           << "}";

        msg.data = ss.str();
        state_pub_.publish(msg);
        last_state_pub_time_ = ros::Time::now();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "on_track_station_control");

    BaseStationController controller;
    controller.spin();

    return 0;
}
