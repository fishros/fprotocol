"""
FProtocol - 轻量级嵌入式/主机之间应用层通信协议

这是一个低代码平台，定义协议文件可生成C++/Python代码文件，导入即用。
"""

__version__ = "1.0.0"
__author__ = "FishROS Team"
__email__ = ""

# 导入主要的类和函数
from .fprotocol import (
    DynamicStruct,
    CircularBuffer,
    FProtocolHeader,
    FProtocolType,
    FProtocol,
)

__all__ = [
    "DynamicStruct",
    "CircularBuffer", 
    "FProtocolHeader",
    "FProtocolType",
    "FProtocol",
]