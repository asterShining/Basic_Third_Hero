#!/bin/bash

set -e  # 遇到错误立即退出

# 配置参数 - Windows 环境适配
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    # Windows 环境
    JLINK_EXE="JLink.exe"  # Windows 下的 J-Link 执行文件
else
    # Linux 环境
    JLINK_EXE="JLinkExe"
fi

DEVICE="STM32F407IGHx" # 芯片型号
INTERFACE="SWD"        # 接口类型
SPEED="4000"           # 通信速度 (kHz)
TARGET_ELF="build/basic_framework.elf"  # 要烧录的 ELF 文件

# 颜色定义 (Windows 兼容)
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    # Windows 终端可能不支持 ANSI 颜色，使用简单提示
    GREEN=""
    RED=""
    YELLOW=""
    NC=""
else
    GREEN='\033[0;32m'
    RED='\033[0;31m'
    YELLOW='\033[1;33m'
    NC='\033[0m'
fi

echo "=== J-Link 烧录脚本 (Windows) ==="
echo "设备: $DEVICE"
echo "接口: $INTERFACE"
echo "速度: $SPEED kHz"
echo "目标文件: $TARGET_ELF"
echo

# 检查 J-Link 是否可用
if ! command -v "$JLINK_EXE" &> /dev/null; then
    echo "错误: 未找到 J-Link 可执行文件 '$JLINK_EXE'"
    echo "请检查:"
    echo "  1. J-Link 软件是否已安装"
    echo "  2. J-Link 安装目录是否已添加到系统 PATH"
    echo "  3. 或者尝试使用完整路径，如: '/c/Program Files/SEGGER/JLink/JLink.exe'"
    exit 1
fi

# 检查目标文件是否存在
if [ ! -f "$TARGET_ELF" ]; then
    echo "错误: 未找到目标文件 '$TARGET_ELF'"
    echo "请先编译项目或检查文件路径"
    exit 1
fi

echo "正在连接设备并烧录..."

# 创建临时命令文件（避免 Windows 下 heredoc 问题）
SCRIPT_FILE="jlink_commands.jlink"
cat > "$SCRIPT_FILE" << EOF
device $DEVICE
if $INTERFACE
speed $SPEED
connect
halt
loadfile $TARGET_ELF
r
g
qc
EOF

# 执行烧录
if "$JLINK_EXE" -CommandFile "$SCRIPT_FILE"; then
    echo
    echo "✅ 烧录成功！"
    echo "程序已烧录到设备并开始运行"
    # 清理临时文件
    rm -f "$SCRIPT_FILE"
else
    echo
    echo "❌ 烧录失败！"
    echo "请检查:"
    echo "  1. J-Link 设备是否连接并供电"
    echo "  2. 设备型号 $DEVICE 是否正确"
    echo "  3. 接口连接是否可靠"
    echo "  4. 目标文件路径是否正确"
    # 清理临时文件
    rm -f "$SCRIPT_FILE"
    exit 1
fi