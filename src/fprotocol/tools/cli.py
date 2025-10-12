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

# 添加工具目录到路径
current_dir = Path(__file__).parent.parent.parent.parent  # 回到项目根目录
tool_dir = current_dir / "tool"
sys.path.insert(0, str(tool_dir))

try:
    from gencode_c import generate_c_code
    from gencode_py import generate_python_code
except ImportError:
    # 如果无法导入，尝试直接执行脚本
    import subprocess
    import tempfile
    
    def generate_c_code(input_file, output_directory):
        """通过子进程调用生成C代码"""
        script_path = tool_dir / "gencode_c.py"
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"生成C代码失败: {result.stderr}")
    
    def generate_python_code(input_file, output_directory):
        """通过子进程调用生成Python代码"""
        script_path = tool_dir / "gencode_py.py"
        result = subprocess.run([
            sys.executable, str(script_path), input_file, output_directory
        ], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"生成Python代码失败: {result.stderr}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="FProtocol 代码生成工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  fprotocol_tool gencode example.fproto --type=c --out_dir=./output
  fprotocol_tool gencode example.fproto --type=cpp --out_dir=./output
  fprotocol_tool gencode example.fproto --type=py --out_dir=./output
  fprotocol_tool gencode example.fproto --type=py --out_dir=.
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # gencode 子命令
    gencode_parser = subparsers.add_parser('gencode', help='生成代码')
    gencode_parser.add_argument('proto_file', help='.fproto 协议文件路径')
    gencode_parser.add_argument('--type', choices=['c', 'cpp', 'py'], required=True,
                               help='生成代码类型: c, cpp 或 py')
    gencode_parser.add_argument('--out_dir', required=True,
                               help='输出目录')
    
    args = parser.parse_args()
    
    if args.command == 'gencode':
        handle_gencode(args)
    else:
        parser.print_help()


def handle_gencode(args):
    """处理代码生成命令"""
    proto_file = Path(args.proto_file)
    out_dir = Path(args.out_dir)
    
    # 检查输入文件是否存在
    if not proto_file.exists():
        print(f"错误: 协议文件 '{proto_file}' 不存在")
        sys.exit(1)
    
    # 创建输出目录
    out_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        if args.type in ['c', 'cpp']:
            generate_c_code(str(proto_file), str(out_dir), args.type)
            print(f"成功生成{args.type.upper()}代码到: {out_dir}")
        elif args.type == 'py':
            generate_python_code(str(proto_file), str(out_dir))
            print(f"成功生成Python代码到: {out_dir}")
    except Exception as e:
        print(f"生成代码时出错: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
