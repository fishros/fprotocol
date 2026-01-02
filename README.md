# FProtocol 

轻量级嵌入式/主机之间应用层通信协议，低代码平台，定义协议文件可生成C++/Python代码文件，导入即用。

## 安装

### 通过pip安装（推荐）

```bash
pip install git+https://github.com/fishros/fprotocol.git
```

### 本地开发安装

```bash
git clone https://github.com/fishros/fprotocol.git
cd fprotocol
pip install -e .
```

## 使用方法

### Python API

```python
import fprotocol

# 创建协议实例
protocol = fprotocol.FProtocol(read_callback, write_callback)

# 添加其他节点
protocol.add_other_node(node_id, node_proto)

# 设置自己的节点
protocol.set_self_node(node_id, node_proto)

# 主循环
while True:
    protocol.tick()
```

### 命令行代码生成工具

安装后可以使用 `fprotocol_tool` 命令生成代码：

```bash
# 生成C代码
fprotocol_tool gencode example.fproto --type=c --out_dir=./output

# 生成C++代码
fprotocol_tool gencode example.fproto --type=cpp --out_dir=./output

# 生成Python代码  
fprotocol_tool gencode example.fproto --type=py --out_dir=./output


fprotocol_tool gencode example.fproto --type=py_ros2 --out_dir=./output

# 生成到当前目录
fprotocol_tool gencode example.fproto --type=py --out_dir=.
```

### 协议文件格式

编写 `.fproto` 文件，参考 `tool/Example.fproto` 格式：

```
# 数据结构定义部分
#数据结构名称,包含类型，名称,数据长度
canframe_t,uint32_t,arbitration_id,
canframe_t,uint8_t,dlc,
canframe_t,uint8_t,rtr,
canframe_t,uint8_t,data,8
---
# 地址 , 类型    ,  变量名,  ,是否回调
0x0001,canframe_t,can_recv_frame,1
0x0002,canframe_t,can_send_frame,1
```

支持的基础类型：
- `uint32_t`, `uint16_t`, `uint8_t`
- `int32_t`, `int16_t`, `int8_t` 
- `float`, `double`, `char`

### 生成文件说明

- **C代码**: 生成 `ExampleProto.h` 和 `ExampleProto.c` 文件，同时拷贝 `fprotocol.h` 和 `fprotocol.c`
- **C++代码**: 生成 `ExampleProto.h` 和 `ExampleProto.cpp` 文件，同时拷贝 `fprotocol.h` 和 `fprotocol.cpp`
- **Python代码**: 生成 `example_proto.py` 文件，包含完整的协议类定义

- C/C++文件名和类名使用大写字母开头（如 `ExampleProto.h`）
- Python文件名使用下划线加小写命名（如 `example_proto.py`），但类名仍使用大写开头

## 平台支持

- ✅ Windows
- ✅ Linux
- ✅ macOS

## 许可证

商业需授权

