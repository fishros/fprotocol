#include "fprotocol.hpp"
#include "RobotProto.hpp"
#include <iostream>
#include <memory>

using namespace FProtocol;

// 全局变量
std::unique_ptr<Handler> handler;
RobotProtocol robot_protocol;

// Mock读取函数
int32_t mock_read(int16_t from, uint8_t *buf, int32_t size) {
    return 0;
}

// Mock写入函数
int32_t mock_write(int16_t to, uint8_t *buf, int32_t size) {
    std::printf("Write data (size=%d): ", size);
    for (int i = 0; i < size; i++) {
        std::printf("0x%02X ", buf[i]);
    }
    std::printf("\n");
    
    // 将写出的数据回环到接收handler的接收缓冲区，模拟网络响应
    handler->readPut(buf, size);
    return size;
}

// LED控制回调函数类
class RobotController {
public:
    int16_t handle_ctrl_led_callback(uint16_t type, uint32_t from, uint16_t error_code) {
        std::printf("\n=== ctrl_led callback called ===\n");
        std::printf("Type: 0x%02X ", type);
        
        if (type == static_cast<uint16_t>(ProtocolType::SERVICE_REQUEST_READ)) {
            std::printf("(SERVICE_REQUEST_READ)\n");
        } else if (type == static_cast<uint16_t>(ProtocolType::SERVICE_RESPONSE_READ)) {
            std::printf("(SERVICE_RESPONSE_READ)\n");
        } else if (type == static_cast<uint16_t>(ProtocolType::TRANSPORT_DATA)) {
            std::printf("(TRANSPORT_DATA)\n");
        } else {
            std::printf("(Other)\n");
        }
        
        std::printf("From: 0x%02X\n", from);
        std::printf("Error code: 0x%04X\n", error_code);
        std::printf("LED value: %d\n", robot_protocol.get_ctrl_led());
        std::printf("==========================\n\n");
        return 0;
    }
    
    int16_t handle_ctrl_beep_callback(uint16_t type, uint32_t from, uint16_t error_code) {
        std::printf("\n=== ctrl_beep callback called ===\n");
        std::printf("Type: 0x%02X, From: 0x%02X, Error: 0x%04X\n", type, from, error_code);
        std::printf("Beep value: %d\n", robot_protocol.get_ctrl_beep());
        std::printf("==========================\n\n");
        return 0;
    }
};

int main() {
    std::cout << "=== Testing C++ Robot Protocol ===\n\n";
    
    // 创建handler
    handler = std::make_unique<Handler>(mock_read, mock_write);
    
    // 创建控制器对象
    RobotController controller;
    
    // 设置初始LED值
    robot_protocol.set_ctrl_led(0x55);
    robot_protocol.set_ctrl_beep(0xAA);
    
    // 设置回调函数 - 使用成员函数
    robot_protocol.set_ctrl_led_callback(&controller, &RobotController::handle_ctrl_led_callback);
    robot_protocol.set_ctrl_beep_callback(&controller, &RobotController::handle_ctrl_beep_callback);
    
    // 设置自身节点
    handler->setSelfNode(0x01, robot_protocol.getIndexInfoFunction());
    
    std::cout << "1. Testing read_ctrl_led function:\n";
    robot_protocol.read_ctrl_led(handler.get(), 0x01);
    handler->tick();
    handler->tick();
    
    std::cout << "\n2. Testing write_ctrl_led function:\n";
    robot_protocol.set_ctrl_led(0x99);
    robot_protocol.write_ctrl_led(handler.get(), 0x01, false);  // TRANSPORT_DATA
    handler->tick();
    handler->tick();
    
    std::cout << "\n3. Testing read_ctrl_beep function:\n";
    robot_protocol.read_ctrl_beep(handler.get(), 0x01);
    handler->tick();
    handler->tick();
    
    std::cout << "\n4. Testing write_ctrl_beep function:\n";
    robot_protocol.set_ctrl_beep(0x77);
    robot_protocol.write_ctrl_beep(handler.get(), 0x01, false);  // TRANSPORT_DATA
    handler->tick();
    handler->tick();
    
    std::cout << "\n5. Testing lambda callback:\n";
    // 也可以使用lambda表达式设置回调
    robot_protocol.set_ctrl_led_callback([](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t {
        std::printf("Lambda callback - Type: 0x%02X, LED: %d\n", type, robot_protocol.get_ctrl_led());
        return 0;
    });
    
    robot_protocol.read_ctrl_led(handler.get(), 0x01);
    handler->tick();
    handler->tick();
    
    std::cout << "\n=== Test completed ===\n";
    return 0;
}