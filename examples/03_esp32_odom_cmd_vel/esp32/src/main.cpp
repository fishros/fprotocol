#include <Arduino.h>
#include "RobotProto.h"

#define LED_PIN 21 // IO21用于LED控制

// FProtocol相关
fprotocol_handler* fprotocol_handler_ptr;

// 心跳相关
unsigned long last_heartbeat_time = 0;
const unsigned long heartbeat_interval = 1000; // 1秒间隔

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

// LED控制回调函数
int16_t callback_led(uint16_t type, uint32_t from, uint16_t error_code) {
  if (led == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  // 去掉串口打印
  return 0;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  // 配置 LED 引脚
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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
  
  // 发送心跳包
  unsigned long current_time = millis();
  if (current_time - last_heartbeat_time >= heartbeat_interval) {
    fprotocol_heart_ping(fprotocol_handler_ptr); // 发送心跳包
    last_heartbeat_time = current_time;
  }
  
  delay(10); 
}


