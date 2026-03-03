#include <Arduino.h>
#include <string.h>
#include "RobotProto.h"

// FProtocol相关
fprotocol_handler* fprotocol_handler_ptr;

// 心跳相关
unsigned long last_heartbeat_time = 0;
const unsigned long heartbeat_interval = 1000; // 1秒间隔

// 导航目标上报相关
unsigned long last_nav_time = 0;
const unsigned long nav_interval = 1000; // 1秒后发送一次
bool nav_sent = false;

typedef struct {
  uint8_t len;
  uint16_t data_size;
  char data[512];
} __attribute__((packed)) nav_target_payload_t;

// 串口读回调
int32_t fprotocol_read_callback(int16_t from, uint8_t *buf, int32_t size) {
  if(Serial.available() > 0) {
    return Serial.readBytes(buf, Serial.available());
  }
  // 无数据可读
  return 0;
}

// 串口写回调
int32_t fprotocol_write_callback(int16_t to, uint8_t *buf, int32_t size) {
  return  Serial.write(buf, size);
}

int16_t callback_cmd_vel(uint16_t type, uint8_t from, uint16_t error_code)
{

  return 0;
}

void send_nav_target_json(uint8_t wid)
{
  nav_target._data_size = sprintf(nav_target.data, "{\"opt\":\"nav\",\"wid\":%u}", wid);
  write_nav_target(fprotocol_handler_ptr, 0xFF, 0);
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  // 初始化FProtocol
  fprotocol_handler_ptr = fprotocol_init(fprotocol_read_callback, fprotocol_write_callback);
  // 设置自己的节点ID
  fprotocol_set_self_node(fprotocol_handler_ptr, 0x01, robot_index_info);

}

void loop() {
  // FProtocol主循环
  if (fprotocol_handler_ptr != NULL) {
    fprotocol_tick(fprotocol_handler_ptr);
  }
  
  unsigned long current_time = millis();
  
  // 发送心跳包
  if (current_time - last_heartbeat_time >= heartbeat_interval) {
    fprotocol_heart_ping(fprotocol_handler_ptr, 0xFF); // 广播心跳包
    last_heartbeat_time = current_time;
  }

  // 上报导航目标点JSON（应用层定义）
  if (!nav_sent && current_time - last_nav_time >= nav_interval) {
    send_nav_target_json(12);
    nav_sent = true;
    last_nav_time = current_time;
  }
  
  delay(1); // 减少延迟以提高响应性
}
