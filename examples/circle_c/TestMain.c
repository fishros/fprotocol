#include "fprotocol.h"
#include "RobotProto.h"
fprotocol_handler *handler;

int32_t mock_read(int16_t from, uint8_t *buf, int32_t size) {
    return 0;
}



int32_t mock_write(int16_t from, uint8_t *buf, int32_t size) {
    printf("Write data (size=%d): ", size);
    for (int i = 0; i < size; i++) {
        printf("0x%02X ", buf[i]);
    }
    printf("\n");
    // 将写出的数据回环到接收handler的接收缓冲区，模拟网络响应
    fprotocol_read_put(handler, buf, size);
    return size;
}

// LED控制回调函数 - 用于验证read_led是否成功
int16_t callback_led(uint16_t type, uint32_t from, uint16_t error_code) {
    printf("\n=== callback_led called ===\n");
    printf("Type: 0x%02X ", type);
    // if (type == SERVICE_REQUEST_READ) {
    //     printf("(SERVICE_REQUEST_READ)\n");
    // } else if (type == SERVICE_RESPONSE_READ) {
    //     printf("(SERVICE_RESPONSE_READ)\n");
    // } else {
    //     printf("(Other)\n");
    // }
    // printf("From: 0x%02X\n", from);
    // printf("Error code: 0x%04X\n", error_code);
    printf("LED value: %d\n", led);
    printf("==========================\n\n");
    return 0;
}

int main() {
    printf("=== Testing read_led function ===\n\n");
    
    // 创建两个handler模拟两个节点
    // handler作为接收者（节点0x01）
    handler = fprotocol_init(mock_read, mock_write);
    led = 0x55;
    fprotocol_set_self_node(handler, 0x01, robot_index_info);

    read_led(handler, 0x01);
    fprotocol_tick(handler);
    fprotocol_tick(handler);
    // 清理资源
    fprotocol_delete(handler);
    
    return 0;
}
