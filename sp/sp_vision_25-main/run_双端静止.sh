·#!/bin/bash
# ============================================================
# 漂移诊断实验 - 一键运行脚本
# 用法: bash run_drift_experiment.sh [config_path]
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_DIR/build"
CONFIG="${1:-configs/standard3.yaml}"
LOG_DIR="$PROJECT_DIR/logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/drift_exp_${TIMESTAMP}.log"

echo "==================================================="
echo " 漂移诊断实验"
echo " 时间: $(date)"
echo " 配置: $CONFIG"
echo " 日志: $LOG_FILE"
echo "==================================================="

# ---- 1. 创建日志目录 ----
mkdir -p "$LOG_DIR"

# ---- 2. 检查/编译 ----
echo ""
echo "[1/3] 检查编译..."

if [ ! -d "$BUILD_DIR" ]; then
    echo "  build 目录不存在，创建并编译..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake ..
    make -j$(nproc) auto_aim_debug_mpc
elif [ ! -f "$BUILD_DIR/auto_aim_debug_mpc" ]; then
    echo "  可执行文件不存在，编译中..."
    cd "$BUILD_DIR"
    cmake ..
    make -j$(nproc) auto_aim_debug_mpc
else
    # 检查源文件是否更新
    SRC="$PROJECT_DIR/src/auto_aim_debug_mpc.cpp"
    BIN="$BUILD_DIR/auto_aim_debug_mpc"
    if [ "$SRC" -nt "$BIN" ]; then
        echo "  源文件有更新，重新编译..."
        cd "$BUILD_DIR"
        cmake ..
        make -j$(nproc) auto_aim_debug_mpc
    else
        echo "  ✅ 已编译，无需重新编译"
    fi
fi

echo "  ✅ 编译完成"

# ---- 3. 运行实验 ----
echo ""
echo "[2/3] 启动实验程序..."
echo "  📌 请将目标保持静止"
echo "  📌 用手推动云台缓慢左右转动 (±15°)"
echo "  📌 运行 10~20 秒后按 Ctrl+C 或 q 停止"
echo ""
echo "------- 程序输出（同时保存到日志）-------"

cd "$PROJECT_DIR"
"$BUILD_DIR/auto_aim_debug_mpc" "$CONFIG" 2>&1 | tee "$LOG_FILE"

# ---- 4. 实验结束 ----
echo ""
echo "------- 程序结束 -------"

# 统计日志
DRIFT_COUNT=$(grep -c '^\[DRIFT\]' "$LOG_FILE" 2>/dev/null || echo "0")
TARGET_COUNT=$(grep -c 'NO_TARGET' "$LOG_FILE" 2>/dev/null || echo "0")
DATA_COUNT=$((DRIFT_COUNT - TARGET_COUNT))

echo ""
echo "==================================================="
echo " 实验完成"
echo " 日志文件: $LOG_FILE"
echo " 总帧数: $DRIFT_COUNT"
echo "   有目标: $DATA_COUNT 帧"
echo "   无目标: $TARGET_COUNT 帧"
echo "==================================================="

if [ "$DATA_COUNT" -lt 10 ]; then
    echo ""
    echo "⚠️ 有效数据帧不足 10，建议重新实验"
else
    echo ""
    echo "✅ 数据充足，可以分析"
    echo ""
    echo "下一步：让 agent 分析日志，说："
    echo "  \"分析漂移日志 $LOG_FILE\""
fi
