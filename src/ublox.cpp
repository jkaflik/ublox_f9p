#include <serial_driver/serial_driver.hpp>
#include <pthread.h>
#include <GeographicLib/UTMUPS.hpp>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <mutex>
#include "ublox.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr uint8_t kUbxSync1 = 0xB5;
constexpr uint8_t kUbxSync2 = 0x62;
constexpr size_t kUbxHeaderLength = 6;
constexpr size_t kUbxFrameOverhead = 8;
constexpr size_t kMaxExpectedUbxPayloadLength = 4096;
constexpr size_t kMaxSerialReadSize = 512;
constexpr double kMinMotionHeadingSpeed = 0.05;
constexpr auto kConfigAckTimeout = std::chrono::milliseconds(1000);
}

class UBlox::Serial {
public:
    typedef std::vector<uint8_t> Buffer;

    Serial() : owned_ctx(new IoContext(2)), serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx)) {
        data_updated_ = false;
    }

    void *rxThread(void);

    void write(const Buffer &buffer) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        auto port = serial_driver_->port();
        if (!port || !port->is_open()) {
            log("Attempted serial write while disconnected", WARN);
            return;
        }
        port->send(buffer);
    }

    static void *rxThreadHelper(void *context) {
        return ((UBlox::Serial *) context)->rxThread();
    }

    bool validateChecksum(const Buffer &packet);

    pthread_t rx_thread_{};
    std::atomic_bool rx_thread_run_{false};
    bool rx_thread_started_{false};
    NavPacketHandlerFunction packet_handler_;
    LogFunction log_function_;
    std::unique_ptr<IoContext> owned_ctx{};
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    bool data_updated_;
    std::mutex write_mutex_;
    mutable std::mutex ack_mutex_;
    std::condition_variable ack_cv_;
    bool ack_waiting_{false};
    bool ack_received_{false};
    bool ack_success_{false};
    uint8_t ack_message_class_{};
    uint8_t ack_message_id_{};

    void receiveUbxPacket(std::chrono::time_point<std::chrono::steady_clock> &point, Buffer data);

    void calculateChecksum(const Buffer &packet, uint8_t &ck_a, uint8_t &ck_b) const;

    void log(const std::string &message, LogLevel level) const noexcept;

    void prepareAck(uint8_t messageClass, uint8_t messageID);

    bool waitForAck(uint8_t messageClass, uint8_t messageID, std::chrono::milliseconds timeout);

    void receiveAck(bool success, uint8_t messageClass, uint8_t messageID);
};

void *UBlox::Serial::rxThread(void) {
    log("Start receiving data from u-blox", INFO);

    Buffer buffer;
    Buffer readBuffer(kUbxHeaderLength); // enough bytes to parse a UBX header

    while (rx_thread_run_.load()) {
        size_t bytesRead = 0;
        try {
            auto port = serial_driver_->port();
            if (!port || !port->is_open()) {
                if (rx_thread_run_.load()) {
                    log("Serial port closed while receiver thread was active", WARN);
                }
                break;
            }
            bytesRead = port->receive(readBuffer);
        } catch (const std::exception &e) {
            if (rx_thread_run_.load()) {
                log(std::string("Serial receive failed: ") + e.what(), ERROR);
            }
            break;
        }

        buffer.reserve(buffer.size() + bytesRead);
        buffer.insert(buffer.end(), readBuffer.begin(), readBuffer.begin() + bytesRead);

        if (0 == bytesRead && !buffer.empty()) {
            log("Possibly out-of-sync with u-blox. Read timeout in the middle of a frame.", WARN);
            continue;
        }

        if (buffer.empty()) {
            continue;
        }

        while (buffer.size() >= 2) {
            auto sync = std::find(buffer.begin(), buffer.end(), kUbxSync1);
            if (sync == buffer.end()) {
                log("No UBX sync byte in buffer. Dropping buffered noise.", DEBUG);
                buffer.clear();
                break;
            }

            if (sync != buffer.begin()) {
                log("Skipping bytes before UBX sync", DEBUG);
                buffer.erase(buffer.begin(), sync);
            }

            if (buffer.size() < kUbxHeaderLength) {
                break;
            }

            if (buffer[1] != kUbxSync2) {
                log("Invalid UBX sync pair. Skipping first sync byte.", DEBUG);
                buffer.erase(buffer.begin());
                continue;
            }

            const size_t payloadLength = static_cast<size_t>(buffer[4]) |
                                         (static_cast<size_t>(buffer[5]) << 8);
            if (payloadLength > kMaxExpectedUbxPayloadLength) {
                log("UBX payload length too large: " + std::to_string(payloadLength), WARN);
                buffer.erase(buffer.begin());
                continue;
            }

            const size_t totalLength = payloadLength + kUbxFrameOverhead;

            if (totalLength > buffer.size()) {
                log("Not enough data in buffer to parse packetData. Need " + std::to_string(totalLength) +
                    " bytes, have " + std::to_string(buffer.size()), DEBUG);
                const size_t remaining = totalLength - buffer.size();
                readBuffer.resize(std::max(kUbxHeaderLength, std::min(remaining, kMaxSerialReadSize)));
                break;
            }
            readBuffer.resize(kUbxHeaderLength);

            Buffer packetData(buffer.begin(), buffer.begin() + totalLength);

            if (!validateChecksum(packetData)) {
                log("Got ubx packet with invalid checksum", WARN);
                buffer.erase(buffer.begin());
                continue;
            }

            buffer.erase(buffer.begin(), buffer.begin() + totalLength);
            auto packetReceivedTime = std::chrono::steady_clock::now();
            receiveUbxPacket(packetReceivedTime, Buffer(packetData.begin() + 2, packetData.end() - 2));
        }
    }

    log("Stop receiving data from u-blox", INFO);
    return nullptr;
}

bool UBlox::Serial::validateChecksum(const Buffer &packet) {
    uint8_t ck_a;
    uint8_t ck_b;
    calculateChecksum(packet, ck_a, ck_b);
    return ck_a == packet[packet.size() - 2] && ck_b == packet[packet.size() - 1];
}

void UBlox::Serial::calculateChecksum(const UBlox::Serial::Buffer &packet, uint8_t &ck_a, uint8_t &ck_b) const {
    ck_a= 0;
    ck_b= 0;
    for (auto iter = packet.begin() + 2; iter != packet.end() - 2; ++iter) {
        ck_a += *iter;
        ck_b += ck_a;
    }
}

void UBlox::Serial::log(const std::string &message, UBlox::LogLevel level) const noexcept {
    if (!log_function_) {
        return;
    }

    try {
        log_function_(message, level);
    } catch (...) {
    }
}

void UBlox::Serial::prepareAck(uint8_t messageClass, uint8_t messageID) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_waiting_ = true;
    ack_received_ = false;
    ack_success_ = false;
    ack_message_class_ = messageClass;
    ack_message_id_ = messageID;
}

bool UBlox::Serial::waitForAck(uint8_t messageClass, uint8_t messageID, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(ack_mutex_);
    const bool received = ack_cv_.wait_for(lock, timeout, [this, messageClass, messageID] {
        return ack_received_ && ack_message_class_ == messageClass && ack_message_id_ == messageID;
    });

    if (!received) {
        ack_waiting_ = false;
        return false;
    }

    ack_waiting_ = false;
    return ack_success_;
}

void UBlox::Serial::receiveAck(bool success, uint8_t messageClass, uint8_t messageID) {
    {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        if (!ack_waiting_ || ack_message_class_ != messageClass || ack_message_id_ != messageID) {
            return;
        }
        ack_received_ = true;
        ack_success_ = success;
    }
    ack_cv_.notify_all();
}

void UBlox::Serial::receiveUbxPacket(std::chrono::time_point<std::chrono::steady_clock> &point, Buffer data) {
    if (data.size() < 4) {
        log("Received UBX packet shorter than class/id/length", WARN);
        return;
    }

    uint16_t packetID = data[0] << 8 | data[1];

    switch (packetID) {
        case UbxNavPvt::CLASS_ID << 8 | UbxNavPvt::MESSAGE_ID: {
            // substract class, id and length
            if (data.size() - 4 == sizeof(struct UbxNavPvt)) {
                UbxNavPvt msg;
                std::memcpy(&msg, data.data() + 4, sizeof(msg));
                const auto s = std::make_shared<UbxNavPvt const>(msg);
                if (packet_handler_) {
                    try {
                        packet_handler_(point, s);
                    } catch (const std::exception &e) {
                        log(std::string("GPS packet handler failed: ") + e.what(), ERROR);
                    } catch (...) {
                        log("GPS packet handler failed with an unknown exception", ERROR);
                    }
                }
            } else {
                log("size mismatch for PVT message!", WARN);
            }
        }
            break;

        case 0x10 << 8 | 0x02: {
            log("received unsupported esf measurement message", DEBUG);
        }
            break;

        case 0x05 << 8 | 0x01: {
            if (data.size() - 4 == 2) {
                receiveAck(true, data[4], data[5]);
                log("received UBX-ACK-ACK for " + std::to_string(data[4]) + ":" + std::to_string(data[5]), INFO);
            } else {
                log("received UBX-ACK-ACK with invalid payload size", WARN);
            }
        }
            break;
        case 0x05 << 8 | 0x00: {
            if (data.size() - 4 == 2) {
                receiveAck(false, data[4], data[5]);
                log("received UBX-ACK-NAK for " + std::to_string(data[4]) + ":" + std::to_string(data[5]), WARN);
            } else {
                log("received UBX-ACK-NAK with invalid payload size", WARN);
            }
        }
            break;
        default:
            log("received unsupported ubx message: " + std::to_string(packetID), DEBUG);
            break;
    }
}

UBlox::UBlox() : serial_(new Serial()) {
    serial_->packet_handler_ = std::bind(&UBlox::navPacketHandler, this, std::placeholders::_1, std::placeholders::_2);
}

void UBlox::connect(const std::string &port, const uint32_t baudRate) {
    serial_->log_function_ = log_function_;

    if (isConnected()) {
        throw SerialException("Already connected to serial port.");
    }

    try {
        const auto fc = drivers::serial_driver::FlowControl::NONE;
        const auto pt = drivers::serial_driver::Parity::NONE;
        const auto sb = drivers::serial_driver::StopBits::ONE;
        serial_->device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baudRate, fc, pt, sb);
        serial_->serial_driver_->init_port(port, *serial_->device_config_);
        if (!serial_->serial_driver_->port()->is_open())
        {
            serial_->serial_driver_->port()->open();
        }
    } catch (const std::exception &e) {
        std::stringstream ss;
        ss << "Failed to open the serial port " << port << " to the u-blox. " << e.what();
        throw SerialException(ss.str().c_str());
    }

    // start up a monitoring thread
    serial_->rx_thread_run_ = true;
    int result = pthread_create(&serial_->rx_thread_, NULL, &UBlox::Serial::rxThreadHelper, serial_.get());

    if (result != 0) {
        serial_->rx_thread_run_ = false;
        throw SerialException("Failed to start serial rx thread.");
    }
    serial_->rx_thread_started_ = true;
}

void UBlox::disconnect() {
    if (serial_->rx_thread_started_) {
        serial_->rx_thread_run_ = false;
        try {
            auto port = serial_->serial_driver_->port();
            if (port && port->is_open()) {
                port->close();
            }
        } catch (const std::exception &e) {
            log(std::string("Failed to close serial port: ") + e.what(), WARN);
        }
        pthread_join(serial_->rx_thread_, nullptr);
        serial_->rx_thread_started_ = false;
    } else if (isConnected()) {
        serial_->serial_driver_->port()->close();
    }
}

bool UBlox::isConnected() const {
    auto port = serial_->serial_driver_->port();
    return port && port->is_open();
}

UBlox::~UBlox() {
    disconnect();
}

void UBlox::log(const std::string &message, UBlox::LogLevel level) const noexcept {
    if (!log_function_) {
        return;
    }

    try {
        log_function_(message, level);
    } catch (...) {
    }
}

void UBlox::navPacketHandler(const std::chrono::time_point<std::chrono::steady_clock> &time, const UbxNavPvtConstPtr &packet) {
    if (!(packet->flags & 0b0000001)) {
        log("No GNSS fix, skipping message", WARN);
        return;
    }

    if (packet->flags3 & 0b1) {
        log("invalid coordinates, skipping message", DEBUG);
        return;
    }

//    if (gps_state_valid_) {
//        auto time_diff = duration_cast<milliseconds>(header_stamp - last_gps_message).count();
//        uint32_t pvt_diff = msg->iTOW - gps_state_iTOW_;
//
//        double diff = (double) time_diff - (double) pvt_diff;
//
//        // Check, if time was spent since the last packet. If not, the data was already in some buffer somewhere
//        if (time_diff == 0) {
//            log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
//                std::to_string(time_diff), ERROR);
//        } else if (abs(diff) > 100.0) {
//            log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
//                std::to_string(time_diff), ERROR);
//        }
//    }

    auto gpsState = GPSState();

    switch(packet->fixType) {
        case 1:
            gpsState.fix_type = GPSState::FixType::DR_ONLY;
            break;
        case 2:
            gpsState.fix_type = GPSState::FixType::FIX_2D;
            break;
        case 3:
            gpsState.fix_type = GPSState::FixType::FIX_3D;
            break;
        case 4:
            gpsState.fix_type = GPSState::FixType::GNSS_DR_COMBINED;
            break;
        default:
            gpsState.fix_type = GPSState::FixType::NO_FIX;
            break;
    }

    // Calculate the position
    double lat = (double) packet->lat / 10000000.0;
    double lon = (double) packet->lon / 10000000.0;
    double altitude = (double) packet->height / 1000.0;
    double easting, northing;
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
    gpsState.pos_lat = lat;
    gpsState.pos_lon = lon;
    gpsState.pos_altitude = altitude;
    gpsState.position_valid = true;
    gpsState.pos_e = easting;
    gpsState.pos_n = northing;
    gpsState.pos_u = altitude;
    gpsState.position_accuracy = (double) std::sqrt(
            std::pow((double) packet->hAcc / 1000.0, 2) + std::pow((double) packet->vAcc / 1000.0, 2));
    gpsState.horizontal_accuracy = (double) packet->hAcc / 1000.0;
    gpsState.vertical_accuracy = (double) packet->vAcc / 1000.0;

    gpsState.vel_e = packet->velE / 1000.0;
    gpsState.vel_n = packet->velN / 1000.0;
    gpsState.vel_u = -packet->velD / 1000.0;

    gpsState.vel_accuracy = packet->sAcc / 1000.0;

    double headAcc = (packet->headAcc / 100000.0) * (kPi / 180.0);

    double hedVeh = packet->headVeh / 100000.0;
    hedVeh = -hedVeh * (kPi / 180.0);
    hedVeh = std::fmod(hedVeh + kHalfPi, 2.0 * kPi);
    while (hedVeh < 0) {
        hedVeh += kPi * 2.0;
    }

    double headMotion = packet->headMot / 100000.0;
    headMotion = -headMotion * (kPi / 180.0);
    headMotion = std::fmod(headMotion + kHalfPi, 2.0 * kPi);
    while (headMotion < 0) {
        headMotion += kPi * 2.0;
    }

    gpsState.motion_heading_valid = std::abs(packet->gSpeed / 1000.0) > kMinMotionHeadingSpeed;
    gpsState.motion_heading = headMotion;
    gpsState.motion_heading_accuracy = headAcc;

    // headAcc is the same for both
    gpsState.vehicle_heading_valid = packet->flags & UbxNavPvt::FLAGS_HEAD_VEH_VALID;
    gpsState.vehicle_heading_accuracy = headAcc;
    gpsState.vehicle_heading = hedVeh;

    switch (packet->flags & UbxNavPvt::FLAGS_CARRIER_PHASE_MASK) {
        case UbxNavPvt::CARRIER_PHASE_FIXED:
            gpsState.rtk_type = GPSState::RTK_FIX;
            break;
        case UbxNavPvt::CARRIER_PHASE_FLOAT:
            gpsState.rtk_type = GPSState::RTK_FLOAT;
            break;
        default:
            gpsState.rtk_type = GPSState::RTK_NONE;
            break;
    }
    gpsState.differential_solution = packet->flags & UbxNavPvt::FLAGS_DIFF_SOLN;

    gpsState.sensor_time = packet->iTOW;
    gpsState.received_time = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();

    // Latency tracking
//    auto last_gps_message = time;
//    gps_state_valid_ = true;
//    gps_state_iTOW_ = msg->iTOW;

    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    if (gps_state_handler_)
        gps_state_handler_(gpsState);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    if (millis > 10) {
        log("Slow GPS state handler: " + std::to_string(millis) + " ms", ERROR);
    }
}

void UBlox::sendRTCM(const std::vector<uint8_t> &data) {
    if (data.empty()) {
        return;
    }

    log("Sending RTCM message", DEBUG);

    serial_->write(data);
}

void UBlox::setGPSStateCallback(const UBlox::GPSStateHandlerFunction &handler) {
    gps_state_handler_ = handler;
}

void UBlox::setLogCallback(const UBlox::LogFunction &handler) {
    log_function_ = handler;
    serial_->log_function_ = handler;
}

void UBlox::sendPacket(uint8_t messageClass, uint8_t messageID, const std::vector<uint8_t> &payload) {
    // build UBX frame
    std::vector<uint8_t> buffer(payload.size() + 8); // 8 bytes for header and footer

    // sync chars
    buffer[0] = 0xB5;
    buffer[1] = 0x62;
    // message
    buffer[2] = messageClass;
    buffer[3] = messageID;
    // 2 byte length
    buffer[4] = payload.size() & 0xFF;
    buffer[5] = (payload.size() >> 8) & 0xFF;

    std::copy(payload.begin(), payload.end(), buffer.begin() + 6);

    serial_->calculateChecksum(buffer, buffer[buffer.size() - 2], buffer[buffer.size() - 1]);

    return serial_->write(buffer);
}

void UBlox::ConfigSet::set(uint32_t keyID, uint16_t value) {
    appendKeyID(keyID);

    data_.push_back(value & 0xFF);
    data_.push_back((value >> 8) & 0xFF);
}

void UBlox::ConfigSet::set(uint32_t keyID, uint8_t value) {
    appendKeyID(keyID);

    data_.push_back(value);
}

void UBlox::ConfigSet::appendKeyID(uint32_t keyID) {
    data_.push_back(keyID & 0xFF);
    data_.push_back((keyID >> 8) & 0xFF);
    data_.push_back((keyID >> 16) & 0xFF);
    data_.push_back((keyID >> 24) & 0xFF);
}


void UBlox::setConfig(UBlox::ConfigSet set) {
    if (set.data_.size() == 0) {
        log("Attempt to configure with no values", WARN);
        return;
    }

    UbxCfgValSetHeader header{};
    header.layer = UbxCfgValSetHeader::LAYER_RAM;

    std::vector<uint8_t> payload(sizeof(header) + set.data_.size());
    std::copy((uint8_t *) &header, (uint8_t *) &header + sizeof(header), payload.begin());
    std::copy(set.data_.begin(), set.data_.end(), payload.begin() + sizeof(header));

    serial_->prepareAck(UbxCfgValSetHeader::CLASS_ID, UbxCfgValSetHeader::MESSAGE_ID);
    sendPacket(UbxCfgValSetHeader::CLASS_ID, UbxCfgValSetHeader::MESSAGE_ID, payload);
    if (!serial_->waitForAck(UbxCfgValSetHeader::CLASS_ID, UbxCfgValSetHeader::MESSAGE_ID, kConfigAckTimeout)) {
        log("UBX-CFG-VALSET was not acknowledged", WARN);
    }
}
