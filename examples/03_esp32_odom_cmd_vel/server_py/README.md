# FProtocol UDP服务端

这是一个基于FProtocol协议的UDP服务端节点，用于控制ESP32的LED。

## 功能特性

- 通过UDP与ESP32通信
- 支持LED开关控制
- 支持LED状态读取
- 自动心跳检测
- 用户友好的命令行界面

## 环境要求

- Python 3.6+
- ESP32设备连接到WiFi网络

## 网络配置

### ESP32配置
ESP32需要连接到WiFi网络，默认配置：
- WiFi SSID: fishros
- WiFi密码: 88888888
- ESP32本地端口: 8889
- 服务端IP: 192.168.1.101 (需要根据实际情况修改)

### 服务端配置
- 服务端口: 8888
- 监听所有网络接口: 0.0.0.0

如需修改，请编辑 `server.py` 文件中的参数：

```python
server = FProtocolUDPServer(server_port=8888, esp32_ip="192.168.1.100")
```

## 使用方法

### 1. 运行测试脚本

首先运行测试脚本检查环境：

```bash
python test_server.py
```

### 2. 启动服务端

```bash
python server.py
```

### 3. 控制命令

服务端启动后，可以使用以下命令：

- `1` - 开启LED
- `0` - 关闭LED  
- `r` - 读取LED状态
- `h` - 发送心跳包
- `q` - 退出程序

### 节点ID配置

- 服务端节点ID：0x02
- ESP32节点ID：0x01

## 协议说明

### LED控制协议

- 索引：0x0001
- 数据类型：uint8_t
- 值：0=关闭，1=开启

### 消息类型

- SERVICE_REQUEST_WRITE (0x01) - 写入请求
- SERVICE_REQUEST_READ (0x02) - 读取请求
- SERVICE_RESPONSE_WRITE (0x03) - 写入响应
- SERVICE_RESPONSE_READ (0x04) - 读取响应
- HEART_PING (0x07) - 心跳请求
- HEART_PONG (0x08) - 心跳响应

## 故障排除

### 串口连接失败

1. 检查ESP32是否连接到COM14
2. 确认串口未被其他程序占用
3. 检查串口驱动是否正确安装

### 通信失败

1. 确认ESP32运行了对应的FProtocol客户端代码
2. 检查波特率设置是否一致
3. 检查协议版本是否匹配

### 模块导入错误

1. 确认Python路径设置正确
2. 检查fprotocol模块是否在正确位置
3. 运行 `python test_server.py` 进行诊断

## 文件结构

```
server_py/
├── server.py          # 主服务端程序
├── robot_proto.py     # 协议定义
├── Robot.fproto       # 协议文件
├── test_server.py     # 测试脚本
└── README.md          # 说明文档
```