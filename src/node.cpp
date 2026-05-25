#include "node.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

UbloxF9PNode::UbloxF9PNode(const rclcpp::NodeOptions &options) : rclcpp::Node(UBLOX_F9P_NODE_NAME, options) {
    debug = this->declare_parameter<bool>("debug", false);
    if (debug) {
        RCLCPP_WARN(this->get_logger(), "Debugging enabled");

        if (rcutils_logging_set_logger_level(UBLOX_F9P_NODE_NAME, RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
        }
    }

    frame_id_ = this->declare_parameter("frame_id", "gps");
    child_frame_id_ = this->declare_parameter("child_frame_id", frame_id_);
    world_frame_id = this->declare_parameter("world_frame", "map");
    publish_motion_odometry_ = this->declare_parameter("publish.motion_odometry", true);

    navsat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    motion_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps/odom", 10);

    this->rtcm_subscriber_ = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10,
                                                                                std::bind(&UbloxF9PNode::rtcmCallback,
                                                                                          this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UbloxF9PNode started");
    const std::string port = this->declare_parameter("port", "/dev/ttyACM0");
    const int baudrate = this->declare_parameter("baudrate", 921600);

    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to " << port << " at " << baudrate << " baud");

    ublox_ = std::make_unique<UBlox>();
    ublox_->setLogCallback(
            std::bind(&UbloxF9PNode::gpsLogCallback, this, std::placeholders::_1, std::placeholders::_2));
    ublox_->setGPSStateCallback(std::bind(&UbloxF9PNode::gpsStateCallback, this, std::placeholders::_1));
    ublox_->connect(port, baudrate);

    if (this->declare_parameter<bool>("config.enabled", false) == true) {
        int measurement_frequency = this->declare_parameter<int>("config.measurement_frequency", 5);
        int uart_output_rate = this->declare_parameter<int>("config.uart_output_rate", measurement_frequency);

        if (measurement_frequency <= 0 || measurement_frequency > 40) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid config.measurement_frequency: " << measurement_frequency);
            throw std::runtime_error("Invalid config.measurement_frequency");
        }
        if (uart_output_rate < 0 || uart_output_rate > 255) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid config.uart_output_rate: " << uart_output_rate);
            throw std::runtime_error("Invalid config.uart_output_rate");
        }

        uint16_t rate_meas = static_cast<uint16_t>(1000 / measurement_frequency);
        uint8_t nav_pvt_uart1 = static_cast<uint8_t>(uart_output_rate);

        UBlox::ConfigSet set;
        set.set(UBlox::ConfigSet::Key::CFG_RATE_MEAS, rate_meas);
        set.set(UBlox::ConfigSet::Key::CFG_MSGOUT_UBX_NAV_PVT_UART1, nav_pvt_uart1);
        ublox_->setConfig(set);

        RCLCPP_INFO_STREAM(this->get_logger(), "Configured UBlox F9P with measurement period " << rate_meas << "ms and UART1 output rate " << static_cast<int>(nav_pvt_uart1) << "Hz");
    }
}

void UbloxF9PNode::gpsLogCallback(const std::string &msg, UBlox::LogLevel level) {
    switch (level) {
        case UBlox::LogLevel::DEBUG:
            if (!debug) {
                return;
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), msg);
            break;
        case UBlox::LogLevel::INFO:
            RCLCPP_INFO_STREAM(this->get_logger(), msg);
            break;
        case UBlox::LogLevel::WARN:
            RCLCPP_WARN_STREAM(this->get_logger(), msg);
            break;
        default:
            RCLCPP_ERROR_STREAM(this->get_logger(), msg);
            break;
    }
}

void UbloxF9PNode::gpsStateCallback(const UBlox::GPSState &state) {
    publishNavSatFix(state);
    publishMotionOdom(state);

    gpsLogCallback("published valid GPS state", UBlox::LogLevel::DEBUG);
}

void UbloxF9PNode::publishNavSatFix(const UBlox::GPSState &state) const {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.latitude = state.pos_lat;
    msg.longitude = state.pos_lon;
    msg.altitude = state.pos_altitude;
    msg.position_covariance = {
            pow(state.horizontal_accuracy, 2), 0.0, 0.0,
            0.0, pow(state.horizontal_accuracy, 2), 0.0,
            0.0, 0.0, pow(state.vertical_accuracy, 2)
    };
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    bool has_fix = false;
    switch (state.fix_type) {
        case UBlox::GPSState::FixType::FIX_2D:
        case UBlox::GPSState::FixType::FIX_3D:
        case UBlox::GPSState::FixType::GNSS_DR_COMBINED:
            has_fix = true;
            break;
        default:
            has_fix = false;
            break;
    }

    if (!has_fix) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (state.rtk_type == UBlox::GPSState::RTK_FIX) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else if (state.rtk_type == UBlox::GPSState::RTK_FLOAT || state.differential_solution) {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    } else {
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }

    navsat_fix_publisher_->publish(msg);
}

void UbloxF9PNode::publishMotionOdom(const UBlox::GPSState &state) const {
    if (!publish_motion_odometry_) {
        return;
    }

    const bool has_heading = state.vehicle_heading_valid || state.motion_heading_valid;
    double heading = 0.0;
    double headingAcc = kPi;

    if (state.vehicle_heading_valid) {
        heading = state.vehicle_heading;
        headingAcc = state.vehicle_heading_accuracy;
    } else if (state.motion_heading_valid) {
        heading = state.motion_heading;
        headingAcc = state.motion_heading_accuracy;
    }

    auto covSpeed = pow(state.vel_accuracy, 2);
    constexpr double kUnknownPoseVariance = 1.0e6;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_id;
    msg.child_frame_id = child_frame_id_;
    msg.twist.twist.linear.x = state.vel_e;
    msg.twist.twist.linear.y = state.vel_n;
    msg.twist.twist.linear.z = state.vel_u;
    msg.twist.covariance[0] = covSpeed;
    msg.twist.covariance[7] = covSpeed;
    msg.twist.covariance[14] = covSpeed;
    msg.twist.covariance[35] = 1.0e6;

    msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    msg.pose.covariance = {
            kUnknownPoseVariance, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, kUnknownPoseVariance, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, kUnknownPoseVariance, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, kUnknownPoseVariance, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, kUnknownPoseVariance, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, has_heading ? pow(headingAcc, 2) : kUnknownPoseVariance,
    };

    motion_odom_publisher_->publish(msg);
}

void UbloxF9PNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
    if (msg->message.empty()) {
        return;
    }

    std::vector<uint8_t> data(msg->message.begin(), msg->message.end());

    ublox_->sendRTCM(data);
}

UbloxF9PNode::~UbloxF9PNode() = default;
