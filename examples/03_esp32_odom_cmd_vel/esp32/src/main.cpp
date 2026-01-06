#include <Arduino.h>
#include "RobotProto.h"

#define LED_PIN 21 // IO21用于LED控制

// FProtocol相关
fprotocol_handler* fprotocol_handler_ptr;

// 心跳相关
unsigned long last_heartbeat_time = 0;
const unsigned long heartbeat_interval = 1000; // 1秒间隔

// Odom上传相关
unsigned long last_odom_time = 0;
const unsigned long odom_interval = 1; // 10ms间隔，即100Hz频率

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

int16_t callback_cmd_vel(uint16_t type, uint32_t from, uint16_t error_code)
{
  return 0;
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

// 更新odom数据（模拟数据）
void update_odom_data() {
  static float time_counter = 0.0;
  time_counter += 0.01; // 100Hz频率，每次增加0.01秒
  
  // 模拟机器人位置和角度变化
  // odom.x = 1.0 * sin(time_counter * 0.5); // x方向正弦运动
  // odom.y = 0.5 * cos(time_counter * 0.3);  // y方向余弦运动
  odom.z = time_counter * 0.1;           // z方向（yaw角度）线性增长
}

void setup() {
  Serial.begin(921600);
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
  
  unsigned long current_time = millis();
  
  // 发送心跳包
  if (current_time - last_heartbeat_time >= heartbeat_interval) {
    fprotocol_heart_ping(fprotocol_handler_ptr, 0x01); // 发送心跳包
    last_heartbeat_time = current_time;
  }
  
  // 以100Hz频率上传odom数据
  // if (current_time - last_odom_time >= odom_interval) {
    update_odom_data(); // 更新odom数据
    write_odom(fprotocol_handler_ptr, 0x01, 0); // 发送odom数据到所有节点
    update_odom_data(); // 更新odom数据
    write_odom(fprotocol_handler_ptr, 0x01, 0); // 发送odom数据到所有节点
    last_odom_time = current_time;
  // }
  
  // delay(1); // 减少延迟以提高响应性
}


