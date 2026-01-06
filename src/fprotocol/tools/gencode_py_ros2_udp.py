#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol Python + ROS2 UDP集成代码生成器
生成基础Python协议代码 + ROS2 UDP集成节点
"""

import csv
import sys
import re
import os

# 尝试相对导入，如果失败则使用绝对导入
try:
    from .gencode_py import generate_python_code
except ImportError:
    # 如果相对导入失败，尝试直接导入或使用subprocess
    try:
        from gencode_py import generate_python_code
    except ImportError:
        import subprocess
        from pathlib import Path
        
        def generate_python_code(input_file, output_directory):
            """通过子进程调用生成Python代码"""
            script_path = Path(__file__).parent / "gencode_py.py"
            result = subprocess.run([
                sys.executable, str(script_path), input_file, output_directory
            ], capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(f"生成Python代码失败: {result.stderr}")

def parse_ros2_mappings(input_file_content):
    """解析ROS2消息接口映射关系"""
    parts = input_file_content.split('---')
    if len(parts) < 3:
        return []  # 没有ROS2映射部分
    
    ros2_mapping_csv = parts[2].strip()
    if not ros2_mapping_csv:
        return []
    
    mappings = []
    csv_reader = csv.reader(ros2_mapping_csv.splitlines())
    for row in csv_reader:
        if len(row) >= 4:
            # 清理每个字段的空白字符
            row = [field.strip() for field in row]
            addr, direction, msg_type, topic_name = row[0], row[1], row[2], row[3]
            mappings.append({
                'addr': addr,
                'direction': direction,  # pub/sub/param/service
                'msg_type': msg_type,
                'topic_name': topic_name
            })
    
    return mappings

def generate_ros2_imports(mappings):
    """生成ROS2相关的import语句"""
    imports = set()
    imports.add("import rclpy")
    imports.add("from rclpy.node import Node")
    
    for mapping in mappings:
        msg_type = mapping['msg_type']
        if '/' in msg_type:
            parts = msg_type.split('/')
            if len(parts) == 3:  # 格式: package/msg/MessageName 或 package/srv/ServiceName
                package, type_dir, type_name = parts
                if type_dir == 'msg':
                    import_stmt = f"from {package}.msg import {type_name}"
                    imports.add(import_stmt)
                elif type_dir == 'srv':
                    import_stmt = f"from {package}.srv import {type_name}"
                    imports.add(import_stmt)
    
    return sorted(list(imports))

def generate_ros2_node_class(file_name, mappings, data_list):
    """生成ROS2节点类"""
    class_name = f"Udp{file_name}Node"
    
    # 生成类定义开始
    content = f"""
class {class_name}(Node):
    def __init__(self, udp_host='0.0.0.0', udp_port=8888):
        super().__init__('{file_name.lower()}_udp_node')
        
        # 初始化UDP套接字
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((udp_host, udp_port))
            self.sock.setblocking(False)  # 设置为非阻塞模式
            self.get_logger().info(f"UDP服务器启动成功，监听地址: {{udp_host}}:{{udp_port}}")
        except Exception as e:
            self.get_logger().error(f"UDP服务器启动失败: {{e}}")
            exit(1)
        
        # 存储客户端地址
        self.client_address = None
        
        # 初始化协议处理器
        self.handler = FProtocol(self.read_callback, self.write_callback)
        self.{file_name.lower()}_proto = {file_name}Proto()
        
        # 设置协议节点
        self.handler.add_other_node(0x0001, self.{file_name.lower()}_proto)
"""
    
    # 生成发布者和订阅者
    publishers = []
    subscribers = []
    
    if mappings:  # 如果有ROS2映射配置
        for mapping in mappings:
            direction = mapping['direction']
            topic_name = mapping['topic_name']
            msg_type = mapping['msg_type']
            
            if direction == 'pub':
                # 提取消息类名
                msg_class = msg_type.split('/')[-1]
                publishers.append((topic_name, msg_class, mapping['addr']))
                content += f"        self.{topic_name}_pub = self.create_publisher({msg_class}, '{topic_name}', 10)\n"
            
            elif direction == 'sub':
                msg_class = msg_type.split('/')[-1]
                subscribers.append((topic_name, msg_class))
                content += f"        self.{topic_name}_sub = self.create_subscription({msg_class}, '{topic_name}', self.{topic_name}_callback, 10)\n"
        
        # 设置协议回调函数 - 自动为发布者设置回调
        content += "\n        # 设置协议回调函数（自动连接到ROS2发布者）\n"
        for topic_name, msg_class, addr in publishers:
            # 根据地址找到对应的协议变量
            protocol_var = None
            for data_addr, data_type, var_name, callback_flag in data_list:
                # 处理地址格式匹配（十六进制 vs 十进制）
                if data_addr.startswith('0x'):
                    data_addr_int = int(data_addr, 16)
                else:
                    data_addr_int = int(data_addr)
                
                if str(addr).startswith('0x'):
                    addr_int = int(addr, 16)
                else:
                    addr_int = int(addr)
                
                if data_addr_int == addr_int:
                    protocol_var = var_name
                    break
            
            if protocol_var:
                content += f"        self.{file_name.lower()}_proto.{protocol_var}.callback = self.{protocol_var}_callback\n"
    else:
        # 没有ROS2映射时，创建基本的示例发布者和订阅者
        content += """
        # 示例：创建基本的发布者和订阅者（请根据需要修改）
        # self.example_pub = self.create_publisher(String, 'example_topic', 10)
        # self.example_sub = self.create_subscription(String, 'example_input', self.example_callback, 10)
        
        # 设置协议回调函数示例
"""
        for addr, data_type, var_name, callback_flag in data_list:
            if len(callback_flag) > 0 and callback_flag[0] == '1':
                content += f"        # self.{file_name.lower()}_proto.{var_name}.callback = self.{var_name}_callback\n"
    
    # 添加定时器和示例控制逻辑
    if not mappings:
        content += f"""
        # 示例：可以在这里添加其他控制逻辑
        pass
"""
    
    # 添加定时器
    content += """
        # 用定时器驱动协议处理
        self.timer = self.create_timer(0.001, self.timer_cb)  # 1ms tick一次
        
        self.get_logger().info("等待UDP客户端连接...")
        self.get_logger().info("-" * 50)
"""
    
    return content, publishers, subscribers

def generate_callback_methods(file_name, mappings, data_list):
    """生成回调方法"""
    content = ""
    
    if mappings:  # 如果有ROS2映射配置
        # 生成协议数据回调 - 专门为发布者生成
        for mapping in mappings:
            if mapping['direction'] == 'pub':
                topic_name = mapping['topic_name']
                msg_class = mapping['msg_type'].split('/')[-1]
                addr = mapping['addr']
                
                # 根据地址找到对应的协议变量
                protocol_var = None
                for data_addr, data_type, var_name, callback_flag in data_list:
                    # 处理地址格式匹配（十六进制 vs 十进制）
                    if data_addr.startswith('0x'):
                        data_addr_int = int(data_addr, 16)
                    else:
                        data_addr_int = int(data_addr)
                    
                    if str(addr).startswith('0x'):
                        addr_int = int(addr, 16)
                    else:
                        addr_int = int(addr)
                    
                    if data_addr_int == addr_int:
                        protocol_var = var_name
                        break
                
                if protocol_var:
                    content += f"""
    def {protocol_var}_callback(self, type, from_node, error_code):
        \"\"\"处理{protocol_var}协议数据并自动发布到ROS2话题 {topic_name}\"\"\"
        # 协议数据已更新，现在转换并发布到ROS2
        msg = {msg_class}()
        now = self.get_clock().now().to_msg()
        
        # TODO: 将协议数据转换为ROS2消息
        # 协议数据可通过 self.{file_name.lower()}_proto.{protocol_var} 访问
        # 示例：
        # if hasattr(self.{file_name.lower()}_proto.{protocol_var}, 'x'):
        #     msg.pose.pose.position.x = float(self.{file_name.lower()}_proto.{protocol_var}.x)
        # msg.header.stamp = now
        # msg.header.frame_id = "base_link"
        
        self.{topic_name}_pub.publish(msg)
        self.get_logger().info(f"发布{topic_name}数据")
        return 0
"""
        
        # 生成ROS2订阅回调
        for mapping in mappings:
            if mapping['direction'] == 'sub':
                topic_name = mapping['topic_name']
                msg_class = mapping['msg_type'].split('/')[-1]
                addr = mapping['addr']
                
                # 根据地址找到对应的协议变量
                protocol_var = None
                for data_addr, data_type, var_name, callback_flag in data_list:
                    # 处理地址格式匹配（十六进制 vs 十进制）
                    if data_addr.startswith('0x'):
                        data_addr_int = int(data_addr, 16)
                    else:
                        data_addr_int = int(data_addr)
                    
                    if str(addr).startswith('0x'):
                        addr_int = int(addr, 16)
                    else:
                        addr_int = int(addr)
                    
                    if data_addr_int == addr_int:
                        protocol_var = var_name
                        break
                
                content += f"""
    def {topic_name}_callback(self, msg):
        \"\"\"处理ROS2 {topic_name}话题订阅并发送到设备\"\"\"
        # TODO: 将ROS2消息转换为协议数据并发送
        # 示例：
        # if hasattr(msg, 'linear') and hasattr(self.{file_name.lower()}_proto, '{protocol_var}'):
        #     self.{file_name.lower()}_proto.{protocol_var}.x = msg.linear.x
        #     self.{file_name.lower()}_proto.{protocol_var}.y = msg.linear.y
        #     self.{file_name.lower()}_proto.{protocol_var}.z = msg.angular.z
        #     self.{file_name.lower()}_proto.write_{protocol_var}(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)
        
        self.get_logger().info(f"收到{topic_name}消息，已发送到设备")
"""
    else:
        # 没有ROS2映射时，生成示例回调方法
        content += """
    # 示例回调方法（请根据实际需求修改）
    # def example_callback(self, msg):
    #     \"\"\"处理ROS2订阅消息\"\"\"
    #     self.get_logger().info(f"收到消息: {msg.data}")
    #     pass
"""
        
        # 为有回调的协议数据生成示例回调
        for addr, data_type, var_name, callback_flag in data_list:
            if len(callback_flag) > 0 and callback_flag[0] == '1':
                content += f"""
    def {var_name}_callback(self, type, from_node, error_code):
        \"\"\"处理{var_name}协议数据回调\"\"\"
        self.get_logger().info(f"收到{var_name}数据回调，类型: {{type}}, 来源: {{from_node}}")
        # TODO: 在这里添加具体的数据处理逻辑
        # 协议数据可通过 self.{file_name.lower()}_proto.{var_name} 访问
        return 0
"""
    
    return content

def generate_udp_methods():
    """生成UDP读写方法"""
    return """
    def read_callback(self):
        \"\"\"UDP读取回调\"\"\"
        try:
            data, addr = self.sock.recvfrom(1024)
            # 记录客户端地址，用于后续发送数据
            if self.client_address != addr:
                self.client_address = addr
                self.get_logger().info(f"新的UDP客户端连接: {addr}")
            return data
        except socket.error:
            # 非阻塞模式下，没有数据时会抛出异常
            return None
        except Exception as e:
            self.get_logger().error(f"UDP读取错误: {e}")
            return None
    
    def write_callback(self, data):
        \"\"\"UDP写入回调\"\"\"
        try:
            if self.client_address:
                self.sock.sendto(data, self.client_address)
            else:
                self.get_logger().warn("没有可用的UDP客户端地址，无法发送数据")
        except Exception as e:
            self.get_logger().error(f"UDP写入错误: {e}")
    
    def timer_cb(self):
        \"\"\"定时器回调，处理协议tick\"\"\"
        # FProtocol处理
        self.handler.tick()
    
    def destroy_node(self):
        \"\"\"节点销毁时关闭UDP套接字\"\"\"
        if hasattr(self, "sock"):
            self.sock.close()
            self.get_logger().info("UDP套接字已关闭")
        super().destroy_node()
"""

def generate_main_function(file_name):
    """生成main函数"""
    class_name = f"Udp{file_name}Node"
    return f"""
def main():
    rclpy.init()
    
    # 可以通过参数自定义UDP配置
    # node = {class_name}(udp_host='192.168.1.100', udp_port=9999)
    node = {class_name}()  # 使用默认参数
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

def generate_python_ros2_udp_code(input_file, output_directory):
    """生成Python + ROS2 UDP集成代码的主函数"""
    # 首先生成基础的Python协议代码
    generate_python_code(input_file, output_directory)
    
    # 获取文件名
    file_name = os.path.basename(input_file).split('.')[0]
    file_name = file_name[0].upper() + file_name[1:] if file_name else file_name
    
    # 读取并解析输入文件
    with open(input_file, 'r', encoding='utf-8') as file:
        input_file_content = file.read()
    
    # 解析ROS2映射关系
    mappings = parse_ros2_mappings(input_file_content)
    
    # 解析协议数据
    # 删除注释后的内容用于解析
    clean_content = re.sub(r'#.*', '', input_file_content)
    parts = clean_content.split('---')
    
    # 确保至少有两个部分（结构定义和协议数据）
    if len(parts) < 2:
        print("错误: 协议文件格式不正确，缺少协议数据部分")
        return
    
    csv_data = parts[1].strip()
    
    csv_reader = csv.reader(csv_data.splitlines())
    data_list = []
    for row in csv_reader:
        if len(row) >= 3:
            # 清理每个字段的空白字符
            row = [field.strip() for field in row]
            # 确保有4列：index, data_type, var_name, callback_flag
            while len(row) < 4:
                row.append('0')  # 默认不回调
            data_list.append(row)
    
    # 生成ROS2集成代码（无论是否有映射都生成）
    ros2_content = generate_ros2_integration_file(file_name, mappings, data_list)
    
    # 保存ROS2集成文件
    ros2_filename = f"{output_directory}/{file_name.lower()}_ros2_udp.py"
    with open(ros2_filename, 'w', encoding='utf-8') as f:
        f.write(ros2_content)
    
    if mappings:
        print(f"已生成ROS2 UDP集成文件（包含{len(mappings)}个ROS2映射）: {ros2_filename}")
    else:
        print(f"已生成ROS2 UDP集成模板文件（无ROS2映射配置）: {ros2_filename}")

def generate_ros2_integration_file(file_name, mappings, data_list):
    """生成完整的ROS2集成文件内容"""
    
    # 生成文件头部注释
    content = f'''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol UDP服务端 + ROS2 集成
自动生成的{file_name}协议ROS2 UDP集成节点
"""

import socket
import time
from {file_name.lower()}_proto import {file_name}Proto
from fprotocol import *
'''
    
    # 添加ROS2相关导入
    if mappings:
        ros2_imports = generate_ros2_imports(mappings)
        for imp in ros2_imports:
            content += imp + "\n"
        content += "\n"  # 添加空行分隔
    else:
        # 基本的ROS2导入
        content += "import rclpy\n"
        content += "from rclpy.node import Node\n"
        content += "# from std_msgs.msg import String  # 示例消息类型，请根据需要修改\n"
        content += "\n"  # 添加空行分隔
    
    # 生成节点类
    node_content, publishers, subscribers = generate_ros2_node_class(file_name, mappings, data_list)
    content += node_content
    
    # 生成回调方法
    content += generate_callback_methods(file_name, mappings, data_list)
    
    # 生成UDP方法
    content += generate_udp_methods()
    
    # 生成main函数
    content += generate_main_function(file_name)
    
    return content

def main():
    """命令行入口点"""
    if len(sys.argv) != 3:
        print("Usage: python gencode_py_ros2_udp.py <input_file> <output_directory>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_directory = sys.argv[2]
    generate_python_ros2_udp_code(input_file, output_directory)

if __name__ == "__main__":
    main()