#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FProtocol 命令行工具
提供代码生成功能
"""

import argparse
import sys
import os
from pathlib import Path

try:
    from .gencode_c import generate_c_code
    from .gencode_cpp import generate_c_code as generate_cpp_code
    from .gencode_cpp_udp import generate_c_code as generate_cpp_udp_code
    from .gencode_cpp_ros2_udp import generate_c_code as generate_cpp_ros2_udp_code
    from .gencode_py import generate_python_code
    from .gencode_py_ros2 import generate_python_ros2_code
    from .gencode_py_ros2_udp import generate_python_ros2_udp_code
except ImportError:
    import subprocess

    def generate_c_code(input_file, output_directory, code_type='c'):
        script_path = Path(__file__).parent / 'gencode_c.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory, code_type
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??C????: {result.stderr}")

    def generate_cpp_code(input_file, output_directory, code_type='cpp'):
        script_path = Path(__file__).parent / 'gencode_cpp.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory, code_type
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??C++????: {result.stderr}")

    def generate_cpp_udp_code(input_file, output_directory, code_type='cpp'):
        script_path = Path(__file__).parent / 'gencode_cpp_udp.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory, code_type
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??C++ UDP????: {result.stderr}")

    def generate_cpp_ros2_udp_code(input_file, output_directory, code_type='cpp'):
        script_path = Path(__file__).parent / 'gencode_cpp_ros2_udp.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory, code_type
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??C++ ROS2 UDP????: {result.stderr}")

    def generate_python_code(input_file, output_directory):
        script_path = Path(__file__).parent / 'gencode_py.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??Python????: {result.stderr}")

    def generate_python_ros2_code(input_file, output_directory):
        script_path = Path(__file__).parent / 'gencode_py_ros2.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??Python ROS2????: {result.stderr}")

    def generate_python_ros2_udp_code(input_file, output_directory):
        script_path = Path(__file__).parent / 'gencode_py_ros2_udp.py'
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"??Python ROS2 UDP????: {result.stderr}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="FProtocol 代码生成工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  fprotocol_tool gencode example.fproto --type=c --out_dir=./output # 生成C风格代码
  fprotocol_tool gencode example.fproto --type=cpp --out_dir=./output # 生成C++风格代码
  fprotocol_tool gencode example.fproto --type=cpp_c --out_dir=./output # 生成C++后缀的C风格代码
  fprotocol_tool gencode example.fproto --type=cpp_udp --out_dir=./output # 生成C++风格代码并生成测试CMAKE及MAIN代码
  fprotocol_tool gencode example.fproto --type=py --out_dir=./output # 生成Python代码
  fprotocol_tool gencode example.fproto --type=py_ros2_serial --out_dir=./output # 生成Python版本的ROS2节点代码(串口通信)
  fprotocol_tool gencode example.fproto --type=py_ros2_udp --out_dir=./output # 生成Python版本ROS2节点代码(UDP网络通信)
  fprotocol_tool gencode example.fproto --type=py --out_dir=. # 生成Python代码在当前目录
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # gencode 子命令
    gencode_parser = subparsers.add_parser('gencode', help='生成代码')
    gencode_parser.add_argument('proto_file', help='.fproto 协议文件路径')
    gencode_parser.add_argument('--type', choices=['c', 'cpp_c', 'cpp', 'cpp_udp', 'cpp_ros2_udp', 'py', 'py_ros2_serial', 'py_ros2_udp'], required=True,
                               help='生成代码类型: c(C代码), cpp_c(C风格的C++代码), cpp(面向对象C++代码), cpp_udp(C++ UDP集成代码), py(Python代码), py_ros2_serial(Python+ROS2串口集成), py_ros2_udp(Python+ROS2 UDP集成)')
    gencode_parser.add_argument('--out_dir', required=True,
                               help='输出目录')
    
    args = parser.parse_args()
    
    if args.command == 'gencode':
        handle_gencode(args)
    else:
        parser.print_help()


def handle_gencode(args):
    """????????"""
    proto_file = Path(args.proto_file)
    out_dir = Path(args.out_dir)

    if not proto_file.exists():
        print(f"??: ???? '{proto_file}' ???")
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        if args.type == 'c':
            generate_c_code(str(proto_file), str(out_dir), 'c')
            print(f"??C???: {out_dir}")
        elif args.type == 'cpp_c':
            generate_c_code(str(proto_file), str(out_dir), 'cpp')
            print(f"??C???C++???: {out_dir}")
        elif args.type == 'cpp':
            generate_cpp_code(str(proto_file), str(out_dir), 'cpp')
            print(f"??C++???: {out_dir}")
        elif args.type == 'cpp_udp':
            generate_cpp_udp_code(str(proto_file), str(out_dir), 'cpp')
            print(f"??C++ UDP???: {out_dir}")
        elif args.type == 'cpp_ros2_udp':
            generate_cpp_ros2_udp_code(str(proto_file), str(out_dir), 'cpp')
            print(f"??C++ ROS2 UDP???: {out_dir}")
        elif args.type == 'py':
            generate_python_code(str(proto_file), str(out_dir))
            print(f"??Python???: {out_dir}")
        elif args.type == 'py_ros2_serial':
            generate_python_ros2_code(str(proto_file), str(out_dir))
            print(f"??Python+ROS2???????: {out_dir}")
        elif args.type == 'py_ros2_udp':
            generate_python_ros2_udp_code(str(proto_file), str(out_dir))
            print(f"??Python+ROS2 UDP?????: {out_dir}")
    except Exception as e:
        print(f"??????: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
