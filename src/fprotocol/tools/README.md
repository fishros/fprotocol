# FProtocol 代码生成工具

FProtocol提供了多种代码生成工具，支持生成C、C++和Python代码。

## 支持的代码类型

### 1. C代码 (`c`)
- 使用传统的C语言风格
- 生成 `.h` 和 `.c` 文件
- 使用函数指针作为回调
- 手动内存管理

### 2. C风格的C++代码 (`cpp_c`)
- 基于C代码生成器，但输出 `.cpp` 文件
- 保持C语言的编程风格
- 可以在C++项目中使用
- 兼容现有的C代码

### 3. 面向对象的C++代码 (`cpp`)
- 使用现代C++特性
- 生成 `.hpp` 和 `.cpp` 文件
- 使用 `std::function` 作为回调
- 智能指针自动内存管理
- 类封装和命名空间

### 4. Python代码 (`py`)
- 纯Python实现
- 面向对象设计
- 支持回调函数

## 使用方法

### 命令行工具

```bash
# 生成C代码
python -m fprotocol.tools.cli gencode protocol.fproto --type=c --out_dir=./output

# 生成C风格的C++代码
python -m fprotocol.tools.cli gencode protocol.fproto --type=cpp_c --out_dir=./output

# 生成面向对象的C++代码
python -m fprotocol.tools.cli gencode protocol.fproto --type=cpp --out_dir=./output

# 生成Python代码
python -m fprotocol.tools.cli gencode protocol.fproto --type=py --out_dir=./output
```

### 直接调用生成器

```bash
# C代码生成器
python gencode_c.py protocol.fproto ./output c

# C风格的C++代码生成器
python gencode_c.py protocol.fproto ./output cpp

# 面向对象的C++代码生成器
python gencode_cpp.py protocol.fproto ./output cpp
```

## 协议文件格式

协议文件 (`.fproto`) 包含两部分，用 `---` 分隔：

1. **结构体定义**：定义数据结构
2. **协议数据定义**：定义协议索引和变量

### 示例协议文件

```
# 结构体定义
SensorData,uint8_t,sensor_id,
SensorData,uint16_t,temperature,
SensorData,uint16_t,humidity,
SensorData,float,voltage,
MotorCmd,uint8_t,motor_id,
MotorCmd,int16_t,speed,
MotorCmd,uint8_t,direction,
---
# 协议数据定义: index,data_type,var_name,callback_flag
1,SensorData,sensor_data,1
2,MotorCmd,motor_cmd,1
3,uint16_t,status_code,0
4,float,battery_voltage,0
```

## 生成的代码对比

### C代码风格
```c
// 回调函数
int16_t callback_sensor_data(uint16_t type, uint32_t from, uint16_t error_code);

// 读写函数
void write_sensor_data(fprotocol_handler *handler, uint16_t node, uint8_t response);
void read_sensor_data(fprotocol_handler *handler, uint16_t node);

// 获取索引信息
extern fprotocol_get_index_info_t test_protocol_index_info;
```

### 面向对象C++代码风格
```cpp
class TestProtocolProtocol {
public:
    TestProtocolProtocol();
    ~TestProtocolProtocol() = default;

    // 回调函数
    static int16_t callback_sensor_data(uint16_t type, uint32_t from, uint16_t error_code);

    // 读写函数
    void write_sensor_data(Handler* handler, uint16_t node, bool response = false);
    void read_sensor_data(Handler* handler, uint16_t node);

    // 获取索引信息
    static ProtocolData* getIndexInfo(uint16_t index);
    static std::function<ProtocolData*(uint16_t)> getIndexInfoFunction();
};
```

## 主要区别

| 特性 | C代码 | C风格C++ | 面向对象C++ |
|------|-------|----------|-------------|
| 文件扩展名 | .h/.c | .h/.cpp | .hpp/.cpp |
| 内存管理 | 手动 | 手动 | 智能指针 |
| 回调方式 | 函数指针 | 函数指针 | std::function |
| 代码组织 | 全局函数 | 全局函数 | 类封装 |
| 命名空间 | 无 | 无 | FProtocol |
| 现代C++特性 | 无 | 无 | 支持 |

## 选择建议

- **C代码**: 适用于嵌入式系统、资源受限环境
- **C风格C++**: 适用于需要在C++项目中使用但保持C风格的场景
- **面向对象C++**: 适用于现代C++项目，需要更好的封装和类型安全
- **Python代码**: 适用于快速原型开发和测试