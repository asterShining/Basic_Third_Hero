#!/usr/bin/env bash
set -euo pipefail

#----------- 用户可调参数 -----------
BUILD_DIR=${BUILD_DIR:-build}
CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE:-cmake/gcc-arm-none-eabi.cmake}
JOBS=${JOBS:-24}
#------------------------------------

RED='\033[31;1m'; GREEN='\033[32;1m'; NC='\033[0m'
log() { echo -e "${GREEN}[*]${NC} $*"; }
die() { echo -e "${RED}[✗]${NC} $*" >&2; exit 1; }

REPO_ROOT="$(pwd)"                       # 仓库根目录绝对路径
BUILD_ABS="$REPO_ROOT/$BUILD_DIR"        # build 目录绝对路径

log "开始构建项目"
mkdir -p "$BUILD_ABS"

#----------- CMake 配置 ------------
need_reconfigure=0
[[ ! -f "$BUILD_ABS/CMakeCache.txt" ]] ||
[[ "$REPO_ROOT/CMakeLists.txt" -nt "$BUILD_ABS/CMakeCache.txt" ]] ||
[[ "$REPO_ROOT/$CMAKE_TOOLCHAIN_FILE" -nt "$BUILD_ABS/CMakeCache.txt" ]] \
&& need_reconfigure=1

if (( need_reconfigure )); then
    log "运行 CMake 配置..."
    cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE="$REPO_ROOT/$CMAKE_TOOLCHAIN_FILE" -S "$REPO_ROOT" -B "$BUILD_ABS" \
      || die "CMake 配置失败"
else
    log "CMake 配置未变化，跳过 configure"
fi

#----------- 编译 ------------------
log "开始编译（ninja -j$JOBS）..."
ninja -C "$BUILD_ABS" -j "$JOBS" || die "构建失败，请检查上面的错误信息"

#----------- 结果提示 --------------
log "生成编译文件："
find "$BUILD_ABS" -type f \( -name "*.elf" -o -name "*.hex" -o -name "*.bin" \) -printf "  %p\n"
log "构建完成 🎉"