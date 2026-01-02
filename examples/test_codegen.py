#!/usr/bin/env python3
"""
测试代码生成功能的脚本
"""

import os
import sys
import subprocess
from pathlib import Path

def test_code_generation():
    """测试不同类型的代码生成"""
    
    # 获取项目根目录
    project_root = Path(__file__).parent.parent
    tools_dir = project_root / "src" / "fprotocol" / "tools"
    proto_file = Path(__file__).parent / "test_protocol.fproto"
    
    # 测试输出目录
    test_outputs = {
        'c': Path(__file__).parent / "output_c",
        'cpp_c': Path(__file__).parent / "output_cpp_c", 
        'cpp': Path(__file__).parent / "output_cpp"
    }
    
    # 创建输出目录
    for output_dir in test_outputs.values():
        output_dir.mkdir(exist_ok=True)
    
    print("开始测试代码生成...")
    
    # 测试C代码生成
    print("\n1. 测试C代码生成...")
    try:
        result = subprocess.run([
            sys.executable, 
            str(tools_dir / "gencode_c.py"),
            str(proto_file),
            str(test_outputs['c']),
            'c'
        ], capture_output=True, text=True, check=True)
        print("✓ C代码生成成功")
        print(f"输出目录: {test_outputs['c']}")
    except subprocess.CalledProcessError as e:
        print(f"✗ C代码生成失败: {e.stderr}")
    
    # 测试C风格的C++代码生成
    print("\n2. 测试C风格的C++代码生成...")
    try:
        result = subprocess.run([
            sys.executable, 
            str(tools_dir / "gencode_c.py"),
            str(proto_file),
            str(test_outputs['cpp_c']),
            'cpp'
        ], capture_output=True, text=True, check=True)
        print("✓ C风格的C++代码生成成功")
        print(f"输出目录: {test_outputs['cpp_c']}")
    except subprocess.CalledProcessError as e:
        print(f"✗ C风格的C++代码生成失败: {e.stderr}")
    
    # 测试面向对象的C++代码生成
    print("\n3. 测试面向对象的C++代码生成...")
    try:
        result = subprocess.run([
            sys.executable, 
            str(tools_dir / "gencode_cpp.py"),
            str(proto_file),
            str(test_outputs['cpp']),
            'cpp'
        ], capture_output=True, text=True, check=True)
        print("✓ 面向对象的C++代码生成成功")
        print(f"输出目录: {test_outputs['cpp']}")
    except subprocess.CalledProcessError as e:
        print(f"✗ 面向对象的C++代码生成失败: {e.stderr}")
    
    # 列出生成的文件
    print("\n生成的文件:")
    for code_type, output_dir in test_outputs.items():
        print(f"\n{code_type.upper()}代码:")
        if output_dir.exists():
            for file in output_dir.iterdir():
                if file.is_file():
                    print(f"  - {file.name}")
        else:
            print("  (目录不存在)")

if __name__ == "__main__":
    test_code_generation()