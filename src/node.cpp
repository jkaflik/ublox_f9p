#include "node.hpp"

UbloxF9PNode::UbloxF9PNode(const rclcpp::NodeOptions &options) : rclcpp::Node(UBLOX_F9P_NODE_NAME, options) {
    debug = this->declare_parameter("debug", false);
    if (debug) {
        RCLCPP_WARN(this->get_logger(), "Debugging enabled");

        if (rcutils_logging_set_logger_level(UBLOX_F9P_NODE_NAME, RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
        }
    }

    navsat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

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
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";
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

    gpsLogCallback("published valid GPS state", UBlox::LogLevel::DEBUG);

    // todo: handle the rest of GpsState values
}

void UbloxF9PNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
    std::vector<uint8_t> data(&msg->message.data()[0], &msg->message.data()[msg->message.size()]);

    ublox_->sendRTCM(data);
}

UbloxF9PNode::~UbloxF9PNode() {
    delete ublox_;
}
