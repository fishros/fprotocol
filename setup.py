#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
import os

# 读取README文件
def read_readme():
    with open("README.md", "r", encoding="utf-8") as fh:
        return fh.read()

# 读取版本信息
def get_version():
    version_file = os.path.join("src", "fprotocol", "__init__.py")
    if os.path.exists(version_file):
        with open(version_file, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("__version__"):
                    return line.split("=")[1].strip().strip('"').strip("'")
    return "1.0.0"

setup(
    name="fprotocol",
    version=get_version(),
    author="FProtocol Team",
    author_email="",
    description="轻量级嵌入式/主机之间应用层通信协议，低代码平台",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/your-username/fprotocol",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Communications",
        "Topic :: System :: Networking",
    ],
    python_requires=">=3.7",
    install_requires=[],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov",
            "black",
            "flake8",
        ],
    },
    entry_points={
        "console_scripts": [
            "fprotocol_tool=fprotocol.tools.cli:main",
        ],
    },
    include_package_data=True,
    zip_safe=False,
    keywords="protocol, communication, embedded, serial, can",
    project_urls={
        "Bug Reports": "https://github.com/your-username/fprotocol/issues",
        "Source": "https://github.com/your-username/fprotocol",
        "Documentation": "https://github.com/your-username/fprotocol/wiki",
    },
)
