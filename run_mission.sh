#!/bin/bash

# 定义清理函数：脚本退出时自动执行
cleanup() {
    echo "正在关闭所有进程..."
    # 杀掉所有后台子进程
    kill $(jobs -p) 2>/dev/null
    # 强制清理ROS和Gazebo残留
    killall -9 gzserver gzclient rosmaster rosout nodelet 2>/dev/null
    echo ">>> 环境已清理完毕，随时可以再次启动。"
}

# 捕获退出信号 (Ctrl+C)
trap cleanup EXIT

echo "=========================================="
echo "   🤖 课程设计任务一键启动脚本"
echo "=========================================="

# 1. 环境清理
echo "[1/4] 清理旧环境..."
killall -9 gzserver gzclient rosmaster 2>/dev/null
sleep 2

# 2. 加载环境变量
source ~/robot_ws/devel/setup.bash

# 3. 启动仿真环境 (底盘+视觉+Gazebo)
echo "[2/4] 正在启动仿真环境 (Gazebo)..."
# 注意：使用 nohup 或 & 后台运行，并屏蔽日志输出以免刷屏
roslaunch oryxbot_description gazebo.launch > /dev/null 2>&1 &
PID_GAZEBO=$!
echo "      等待 Gazebo 加载 (5秒)..."
sleep 5

# 4. 启动任务层 (Pick AR + 机械臂IK)
echo "[3/4] 正在启动控制节点 (IK & Pick)..."
echo "启动底盘导航与对接功能..."
roslaunch oryxbot_description ar_base_gazebo.launch> /dev/null 2>&1 &
sleep 5

echo "启动机械臂抓取功能..."
roslaunch oryxbot_description pick_ar_gazebo.launch > /dev/null 2>&1 &
sleep 5

echo "启动 IK (逆运动学) 节点..."
rosrun oryxbot_description ik_swiftpro > /dev/null 2>&1 &
sleep 3

echo " 启动摄像头可视化节点..."
rqt_image_view > /dev/null 2>&1 &
sleep 3

# 5. 运行 Python 主控脚本
echo "[4/4] 🚀启动任务控制脚本..."
echo "================ 日志输出 ================"
# 切换到脚本所在目录运行
cd ~/robot_ws/src/oryxbot_description/src/
python3 mission_controller.py

# 脚本运行结束后，等待用户按键再退出
echo "=========================================="
echo "任务脚本已结束。"
echo "按 Enter 键关闭仿真并退出..."
read dummy