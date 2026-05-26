#pragma once

#include <functional>
#include <string>
#include <memory>
#include <chrono>
#include <sstream>
#include <cstdint>

#include "ubx_datatypes.hpp"

class UBlox {
public:
    enum LogLevel {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    struct GPSState {
        enum FixType {
            NO_FIX = 0,
            DR_ONLY = 1,
            FIX_2D = 2,
            FIX_3D = 3,
            GNSS_DR_COMBINED = 4
        };

        enum RTKType {
            RTK_NONE = 0,
            RTK_FLOAT = 1,
            RTK_FIX = 2
        };

        uint32_t sensor_time;
        uint64_t received_time;

        // Position
        bool position_valid;
        // Position accuracy in m
        double position_accuracy;
        double horizontal_accuracy;
        double vertical_accuracy;
        double pos_e, pos_n, pos_u;

        // Pos in lat/lon for VRS
        double pos_lat, pos_lon, pos_altitude;
        uint8_t num_satellites;
        double position_dop;

        // Motion
        bool motion_heading_valid;
        double vel_e, vel_n, vel_u, vel_accuracy;
        double motion_heading_accuracy;
        double motion_heading;

        // Heading
        bool vehicle_heading_valid;
        double vehicle_heading_accuracy;
        // Vehicle heading in rad.
        double vehicle_heading;


        FixType fix_type;
        RTKType rtk_type;
        bool differential_solution;
    };

    class ConfigSet {
    public:
        enum Key {
            CFG_RATE_MEAS = 0x30210001, // uint16, measurement period in ms
            CFG_RATE_NAV = 0x30210002, // uint16, measurements per navigation solution
            CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007, // uint8, output interval on UART1
        };

        void set(const uint32_t keyID, const uint16_t value);
        void set(const uint32_t keyID, const uint8_t value);

        std::vector<uint8_t> data_;
    private:
        void appendKeyID(const uint32_t keyID);
    };

    typedef std::function<void(const GPSState&)> GPSStateHandlerFunction;
    typedef std::function<void(const std::string&, LogLevel level)> LogFunction;

    UBlox();

    ~UBlox();

    void setGPSStateCallback(const GPSStateHandlerFunction& handler);
    void setLogCallback(const LogFunction& handler);

    void connect(const std::string& port, const uint32_t baudRate);

    void disconnect();

    bool isConnected() const;

    void sendPacket(uint8_t messageClass, uint8_t messageID, const std::vector<uint8_t> &payload);

    void sendRTCM(const std::vector<uint8_t>& data);

    void setConfig(UBlox::ConfigSet set);

private:
    typedef std::function<void(const std::chrono::time_point<std::chrono::steady_clock>&, const UbxNavPvtConstPtr&)> NavPacketHandlerFunction;

    void navPacketHandler(const std::chrono::time_point<std::chrono::steady_clock>& time, const UbxNavPvtConstPtr& packet);

    void log(const std::string& message, LogLevel level) const noexcept;

    // Pimpl - hide serial port members from class users
    class Serial;
    std::unique_ptr<Serial> serial_;

    GPSStateHandlerFunction gps_state_handler_;
    LogFunction log_function_;
};

class SerialException : public std::exception
{
    // Disable copy constructors
    SerialException& operator=(const SerialException&);
    std::string e_what_;

public:
    explicit SerialException(const char* description)
    {
        std::stringstream ss;
        ss << "SerialException " << description << " failed.";
        e_what_ = ss.str();
    }
    SerialException(const SerialException& other) : e_what_(other.e_what_)
    {
    }
    virtual ~SerialException() throw()
    {
    }
    virtual const char* what() const throw()
    {
        return e_what_.c_str();
    }
};
