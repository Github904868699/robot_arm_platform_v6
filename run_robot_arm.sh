#!/bin/bash

set -e

WS_DIR="./ros2_ws"

# 检查 ros2_ws 是否存在
if [ ! -d "$WS_DIR" ]; then
    echo "错误: 当前目录下未找到 $WS_DIR 文件夹"
    exit 1
fi

# 检查 build/install/log 是否都存在
if [ -d "$WS_DIR/build" ] && [ -d "$WS_DIR/install" ] && [ -d "$WS_DIR/log" ]; then
    echo ""
else
    echo "Delet build、install、log ..."
    rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
fi

# 进入 ros2_ws 目录
cd "$WS_DIR"

# 编译
echo "colcon build ..."
colcon build

# source 环境
echo "source install/setup.bash ..."
source install/setup.bash

# 启动并保存日志
echo "ros2 launch ..."
ros2 launch robot_arm_bringup_4338 bringup.launch.py use_fake_hardware:=false backend_mode:=real auto_enable_on_activate:=false auto_enable_delay_sec:=1.0 backend_mode:=real 2>&1 | tee real_bringup.log
