# FProtocol 串口服务端使用说明

## 概述
这是FProtocol的串口版本服务端，用于通过串口与ESP32设备通信，控制LED的开关状态。

## 文件说明
- `serial_server.py` - 串口版本的服务端程序
- `robot_proto.py` - 机器人协议定义
- `Robot.fproto` - 协议文件定义

## 使用方法

### 1. 硬件连接
- 将ESP32通过USB连接到电脑
- 确保ESP32的LED连接到GPIO 21引脚

### 2. 配置串口
在 `serial_server.py` 中修改串口配置：
```python
SERIAL_PORT = 'COM3'  # Windows系统，根据实际串口号修改
# 或者 Linux/Mac 系统使用：
# SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际设备路径修改
BAUD_RATE = 115200
```

### 3. 运行服务端
```bash
cd examples/01_esp32_led/server_py
python serial_server.py
```

### 4. 运行ESP32程序
- 使用PlatformIO或Arduino IDE编译并上传ESP32程序
- 确保ESP32程序中的串口波特率设置为115200

## 功能说明
- 服务端每0.3秒发送一次LED状态切换命令
- ESP32接收到命令后会控制LED的开关
- 支持心跳包机制，确保连接稳定
- 自动处理串口连接异常

## 故障排除

### 串口连接失败
1. 检查串口号是否正确
2. 确认ESP32已正确连接
3. 检查串口是否被其他程序占用
4. 尝试更换USB端口

### 通信异常
1. 确认波特率设置一致（115200）
2. 检查USB线缆是否支持数据传输
3. 重启ESP32和服务端程序

## 与UDP版本的区别
- 使用串口通信替代UDP网络通信
- 更稳定的连接，适合本地调试
- 无需网络配置，即插即用
- 通信延迟更低
