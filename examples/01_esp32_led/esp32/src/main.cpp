#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "RobotProto.h"

#define LED_PIN 21 // IO21用于LED控制

// WiFi和UDP配置
const char* ssid = "fishros";
const char* password = "88888888";
const char* udp_server_ip = "192.168.1.101";
const uint16_t udp_server_port = 8888;
WiFiUDP Udp;

// FProtocol相关
fprotocol_handler* fprotocol_handler_ptr;

// UDP缓冲
#define UDP_BUF_SIZE 512
uint8_t udp_recv_buf[UDP_BUF_SIZE];
int udp_recv_len = 0;

// UDP接收缓存指针
int udp_buf_read_pos = 0;

// 心跳相关
unsigned long last_heartbeat_time = 0;
const unsigned long heartbeat_interval = 1000; // 1秒间隔

// LED闪烁相关
unsigned long last_led_time = 0;
const unsigned long led_interval = 500; // 0.5秒间隔
bool led_state = false;

// 连接WiFi函数
void connectToWiFi() {
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// UDP读回调
int32_t fprotocol_read_callback(int16_t from, uint8_t *buf, int32_t size) {
  // 如果buffer空则尝试收新数据包
  if (udp_buf_read_pos >= udp_recv_len) {
    udp_recv_len = 0;
    udp_buf_read_pos = 0;

    int packetSize = Udp.parsePacket();
    if (packetSize > 0 && packetSize <= UDP_BUF_SIZE) {
      udp_recv_len = Udp.read(udp_recv_buf, UDP_BUF_SIZE);
      udp_buf_read_pos = 0;
      Serial.print("[UDP] received packet, size: ");
      Serial.println(udp_recv_len);
    }
  }

  // 从接收缓冲区读取
  if (udp_buf_read_pos < udp_recv_len) {
    int bytes_to_copy = min(size, udp_recv_len - udp_buf_read_pos);
    memcpy(buf, udp_recv_buf + udp_buf_read_pos, bytes_to_copy);
    udp_buf_read_pos += bytes_to_copy;
    return bytes_to_copy;
  }
  // 无数据可读
  return 0;
}

// UDP写回调
int32_t fprotocol_write_callback(int16_t to, uint8_t *buf, int32_t size) {
  // 发送到指定服务器IP和端口
  Udp.beginPacket(udp_server_ip, udp_server_port);
  Udp.write(buf, size);
  Udp.endPacket();

  // 串口打印日志
  Serial.print("[UDP] sent packet, size: ");
  Serial.println(size);
  Serial.print("[UDP] data: ");
  for (int i = 0; i < size; i++) {
    Serial.print(buf[i], HEX); Serial.print(" ");
  }
  Serial.println();

  return size;
}

// LED控制回调函数
int16_t callback_led(uint16_t type, uint32_t from, uint16_t error_code) {
  if (led == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  Serial.print("LED set to: ");
  Serial.println(led ? "ON" : "OFF");
  return 0;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  // 配置 LED 引脚
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  connectToWiFi();

  // UDP客户端初始化, 本地端口可任意
  Udp.begin(8889); // 本地端口8889或其他

  // 初始化FProtocol
  fprotocol_handler_ptr = fprotocol_init(fprotocol_read_callback, fprotocol_write_callback);

  // 设置自己的节点ID
  fprotocol_set_self_node(fprotocol_handler_ptr, 0x01, robot_index_info);

  Serial.println("FProtocol UDP Client Ready!");
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
    Serial.println("[Heartbeat] Sent heartbeat to server");
  }
  
  delay(10); 
}
