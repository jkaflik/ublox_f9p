#include "node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

UbloxF9PNode::UbloxF9PNode(const rclcpp::NodeOptions &options) : rclcpp::Node(UBLOX_F9P_NODE_NAME, options) {
    debug = this->declare_parameter<bool>("debug", false);
    if (debug) {
        RCLCPP_WARN(this->get_logger(), "Debugging enabled");

        if (rcutils_logging_set_logger_level(UBLOX_F9P_NODE_NAME, RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
        }
    }

    frame_id_ = this->declare_parameter("frame_id", "gps");
    child_frame_id_ = this->declare_parameter("child_frame_id", "base_link");

    navsat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

    if (this->declare_parameter<bool>("publish.motion_odometry", false)) {
        motion_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps/odom", 10);
    }

    this->rtcm_subscriber_ = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10,
                                                                                std::bind(&UbloxF9PNode::rtcmCallback,
                                                                                          this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UbloxF9PNode started");
    const std::string port = this->declare_parameter("port", "/dev/ttyACM0");
    const int baudrate = this->declare_parameter("baudrate", 921600);

    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to " << port << " at " << baudrate << " baud");

    ublox_ = new UBlox();
    ublox_->setLogCallback(
            std::bind(&UbloxF9PNode::gpsLogCallback, this, std::placeholders::_1, std::placeholders::_2));
    ublox_->setGPSStateCallback(std::bind(&UbloxF9PNode::gpsStateCallback, this, std::placeholders::_1));
    ublox_->connect(port, baudrate);

    if (this->declare_parameter<bool>("config.enable", false) == true) {
        uint16_t rate_meas = 1000 / this->declare_parameter<uint16_t>("config.measurement_rate", 5);
        uint8_t nav_pvt_uart1 = this->declare_parameter<uint8_t>("config.uart_output_rate", 5);

        UBlox::ConfigSet set;
        set.set(UBlox::ConfigSet::Key::CFG_RATE_MEAS, rate_meas);
        set.set(UBlox::ConfigSet::Key::CFG_MSGOUT_UBX_NAV_PVT_UART1, nav_pvt_uart1);
        ublox_->setConfig(set);

        RCLCPP_INFO_STREAM(this->get_logger(), "Configured UBlox F9P with measurement rate " << rate_meas << "ms and UART1 output rate " << nav_pvt_uart1 << "Hz");
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
            pow(state.position_accuracy, 2), 0.0, 0.0,
            0.0, pow(state.position_accuracy, 2), 0.0,
            0.0, 0.0, pow(state.position_accuracy, 2)
    };
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    navsat_fix_publisher_->publish(msg);
}

void UbloxF9PNode::publishMotionOdom(const UBlox::GPSState &state) const {
    if (!motion_odom_publisher_) {
        return;
    }

    double heading = state.vehicle_heading_valid ? state.vehicle_heading : state.motion_heading;
    double headingAcc = state.vehicle_heading_valid ? state.vehicle_heading_accuracy : state.motion_heading_accuracy;

    auto covSpeed = pow(state.vel_accuracy, 2);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = child_frame_id_;
    msg.twist.twist.linear.x = state.vel_e;
    msg.twist.twist.linear.y = state.vel_n;
    msg.twist.twist.linear.z = state.vel_u;
    msg.twist.covariance[0] = covSpeed;
    msg.twist.covariance[7] = covSpeed;
    msg.twist.covariance[14] = covSpeed;
    msg.twist.covariance[35] = -1;

    msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    msg.pose.covariance = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pow(headingAcc, 2),
    };

    motion_odom_publisher_->publish(msg);
}

void UbloxF9PNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
    std::vector<uint8_t> data(&msg->message.data()[0], &msg->message.data()[msg->message.size()]);

    ublox_->sendRTCM(data);
}

UbloxF9PNode::~UbloxF9PNode() {
    delete ublox_;
}
