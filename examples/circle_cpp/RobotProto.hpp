#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "fprotocol.hpp"
#include <memory>
#include <functional>
#include <cstddef>  // for offsetof

using namespace FProtocol;

typedef struct {
    float x;
    float y;
    float z;
} __attribute__((packed)) float3d_t;

typedef struct {
    float temp;
    float hum;
    uint8_t ladc;
    uint8_t badc;
} __attribute__((packed)) data_t;

typedef struct {
    float x;
    float y;
    float z;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} __attribute__((packed)) imu_t;

class RobotProtocol {
private:
    uint8_t ctrl_led_;
    uint8_t ctrl_beep_;
    float3d_t ctrl_vel_;
    float3d_t sensor_odom_;
    data_t sensor_board_;
    imu_t sensor_imu_;
    uint8_t config_odom_hz_;
    uint8_t config_imu_hz_;
    uint8_t config_sensor_hz_;
    FProtocol::CallbackFunction ctrl_led_callback_;
    FProtocol::CallbackFunction ctrl_beep_callback_;
    FProtocol::CallbackFunction ctrl_vel_callback_;
    FProtocol::CallbackFunction sensor_board_callback_;
    FProtocol::CallbackFunction config_odom_hz_callback_;
    FProtocol::CallbackFunction config_imu_hz_callback_;
    FProtocol::CallbackFunction config_sensor_hz_callback_;

public:
    RobotProtocol();
    ~RobotProtocol() = default;

    const uint8_t& get_ctrl_led() const { return ctrl_led_; }
    void set_ctrl_led(const uint8_t& value) { ctrl_led_ = value; }
    uint8_t* get_ctrl_led_ptr() { return &ctrl_led_; }

    const uint8_t& get_ctrl_beep() const { return ctrl_beep_; }
    void set_ctrl_beep(const uint8_t& value) { ctrl_beep_ = value; }
    uint8_t* get_ctrl_beep_ptr() { return &ctrl_beep_; }

    const float3d_t& get_ctrl_vel() const { return ctrl_vel_; }
    void set_ctrl_vel(const float3d_t& value) { ctrl_vel_ = value; }
    float3d_t* get_ctrl_vel_ptr() { return &ctrl_vel_; }

    const float3d_t& get_sensor_odom() const { return sensor_odom_; }
    void set_sensor_odom(const float3d_t& value) { sensor_odom_ = value; }
    float3d_t* get_sensor_odom_ptr() { return &sensor_odom_; }

    const data_t& get_sensor_board() const { return sensor_board_; }
    void set_sensor_board(const data_t& value) { sensor_board_ = value; }
    data_t* get_sensor_board_ptr() { return &sensor_board_; }

    const imu_t& get_sensor_imu() const { return sensor_imu_; }
    void set_sensor_imu(const imu_t& value) { sensor_imu_ = value; }
    imu_t* get_sensor_imu_ptr() { return &sensor_imu_; }

    const uint8_t& get_config_odom_hz() const { return config_odom_hz_; }
    void set_config_odom_hz(const uint8_t& value) { config_odom_hz_ = value; }
    uint8_t* get_config_odom_hz_ptr() { return &config_odom_hz_; }

    const uint8_t& get_config_imu_hz() const { return config_imu_hz_; }
    void set_config_imu_hz(const uint8_t& value) { config_imu_hz_ = value; }
    uint8_t* get_config_imu_hz_ptr() { return &config_imu_hz_; }

    const uint8_t& get_config_sensor_hz() const { return config_sensor_hz_; }
    void set_config_sensor_hz(const uint8_t& value) { config_sensor_hz_ = value; }
    uint8_t* get_config_sensor_hz_ptr() { return &config_sensor_hz_; }

    void set_ctrl_led_callback(FProtocol::CallbackFunction callback) { ctrl_led_callback_ = callback; }
    template<typename T>
    void set_ctrl_led_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        ctrl_led_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_ctrl_beep_callback(FProtocol::CallbackFunction callback) { ctrl_beep_callback_ = callback; }
    template<typename T>
    void set_ctrl_beep_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        ctrl_beep_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_ctrl_vel_callback(FProtocol::CallbackFunction callback) { ctrl_vel_callback_ = callback; }
    template<typename T>
    void set_ctrl_vel_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        ctrl_vel_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_sensor_board_callback(FProtocol::CallbackFunction callback) { sensor_board_callback_ = callback; }
    template<typename T>
    void set_sensor_board_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        sensor_board_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_config_odom_hz_callback(FProtocol::CallbackFunction callback) { config_odom_hz_callback_ = callback; }
    template<typename T>
    void set_config_odom_hz_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        config_odom_hz_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_config_imu_hz_callback(FProtocol::CallbackFunction callback) { config_imu_hz_callback_ = callback; }
    template<typename T>
    void set_config_imu_hz_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        config_imu_hz_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }
    void set_config_sensor_hz_callback(FProtocol::CallbackFunction callback) { config_sensor_hz_callback_ = callback; }
    template<typename T>
    void set_config_sensor_hz_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        config_sensor_hz_callback_ = [obj, method](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }

    void write_ctrl_led(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_ctrl_led(FProtocol::Handler* handler, uint16_t node);
    void write_ctrl_beep(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_ctrl_beep(FProtocol::Handler* handler, uint16_t node);
    void write_ctrl_vel(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_ctrl_vel(FProtocol::Handler* handler, uint16_t node);
    void write_sensor_odom(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_sensor_odom(FProtocol::Handler* handler, uint16_t node);
    void write_sensor_board(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_sensor_board(FProtocol::Handler* handler, uint16_t node);
    void write_sensor_imu(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_sensor_imu(FProtocol::Handler* handler, uint16_t node);
    void write_config_odom_hz(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_config_odom_hz(FProtocol::Handler* handler, uint16_t node);
    void write_config_imu_hz(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_config_imu_hz(FProtocol::Handler* handler, uint16_t node);
    void write_config_sensor_hz(FProtocol::Handler* handler, uint16_t node, bool response = false);
    void read_config_sensor_hz(FProtocol::Handler* handler, uint16_t node);

    FProtocol::ProtocolData* getIndexInfo(uint16_t index);
    FProtocol::GetIndexInfoFunction getIndexInfoFunction();
};

#endif /* ROBOT_HPP */
