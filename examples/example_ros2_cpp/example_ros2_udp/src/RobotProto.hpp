#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "fprotocol.hpp"
#include <memory>
#include <functional>
#include <cstddef>  // for offsetof

using namespace FProtocol;

typedef struct {
    uint8_t status_charge;
    uint8_t volatge;
} __attribute__((packed)) data_t;

class RobotProtocol {
private:
    uint8_t led_;
    data_t data_;
    FProtocol::CallbackFunction data_callback_;

public:
    RobotProtocol();
    ~RobotProtocol() = default;

    const uint8_t& get_led() const { return led_; }
    void set_led(const uint8_t& value) { led_ = value; }
    uint8_t* get_led_ptr() { return &led_; }

    const data_t& get_data() const { return data_; }
    void set_data(const data_t& value) { data_ = value; }
    data_t* get_data_ptr() { return &data_; }

    void set_data_callback(FProtocol::CallbackFunction callback) { data_callback_ = callback; }
    template<typename T>
    void set_data_callback(T* obj, int16_t (T::*method)(uint16_t, uint32_t, uint16_t)) {
        data_callback_ = [obj, method](uint16_t type, uint8_t from, uint16_t error_code) -> int16_t {
            return (obj->*method)(type, from, error_code);
        };
    }

    void write_led(FProtocol::Handler* handler, uint8_t node, bool response = false);
    void read_led(FProtocol::Handler* handler, uint8_t node);
    void write_data(FProtocol::Handler* handler, uint8_t node, bool response = false);
    void read_data(FProtocol::Handler* handler, uint8_t node);

    FProtocol::ProtocolData* getIndexInfo(uint16_t index);
    FProtocol::GetIndexInfoFunction getIndexInfoFunction();
};

#endif /* ROBOT_HPP */
