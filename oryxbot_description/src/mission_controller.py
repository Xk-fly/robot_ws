#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Pose, Pose2D
from std_srvs.srv import Empty
from arm_controller.srv import PickPlace, move
from relative_move.srv import SetRelativeMove
from ar_pose.srv import Track

# ================= 🔧 坐标校准区域 (请重点关注这里) =================

# --- 1. 场地关键点 (Odom坐标) ---
START_X = 0.034
START_Y = -0.001

# 5号加工中心 (任务一目标)
STATION_5_X = 0.755
STATION_5_Y = 1.162
# 任务一不再使用 ID 对准，直接盲跑过去

# 1号加工中心 (任务二目标)
STATION_1_X = 2.221
STATION_1_Y = 2.144
STATION_1_ID = 1 

# --- 2. 机械臂 Buffer 精确坐标 (单位: mm) ---
# 【你的需求】：外侧(Y大)，X轴较小(更靠后/负值)
# 如果发现抓不到，请按照以下规律微调：
# - 前后偏：改 X (负得越多越靠后)
# - 左右偏：改 Y (180是左侧外围)
BUFFER_X = -80.0       # 向后 80mm (根据"X小"推测是负值)
BUFFER_Y = 180.0       # 向左 180mm (外侧)
BUFFER_Z_GRAB = 25.0   # 抓取高度 (贴合)
BUFFER_Z_LIFT = 150.0  # 抬起安全高度

# --- 3. 中转与交互点 ---
HOME_X = 200.0         # 机械臂回中位置(前方)
HOME_Y = 0.0
HOME_Z = 250.0

TABLE_EXTEND_X = 280.0 # 伸向台子的距离
TABLE_EXTEND_Y = 0.0
TABLE_Z_HOVER = 250.0
TABLE_Z_DROP = 210.0

# ===============================================================

class MissionCommander:
    def __init__(self):
        rospy.init_node('mission_controller', anonymous=True)
        
        rospy.loginfo(">>> 正在连接服务...")
        try:
            rospy.wait_for_service('/relative_move', timeout=20)
            # 任务一不需要 ar_track，但任务二可能需要，保留连接
            rospy.wait_for_service('/ar_track', timeout=20) 
            rospy.wait_for_service('/pick_ar', timeout=20)
            rospy.wait_for_service('/goto_position', timeout=20)
            rospy.wait_for_service('/swiftpro/on', timeout=20)
        except rospy.ROSException:
            rospy.logerr("❌ 服务连接失败！")
            exit(1)
        
        self.srv_move_base = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        self.srv_align = rospy.ServiceProxy('/ar_track', Track)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)
        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)
        
        rospy.loginfo("✅ 机器人就绪。")

    # --- 基础动作 ---

    def move_global(self, tx, ty, use_vision=False, tag_id=0):
        """
        全局导航封装
        use_vision=False: 纯盲跑 (任务一专用)
        use_vision=True:  跑到后进行视觉对准 (任务二专用)
        """
        dx = tx - START_X
        dy = ty - START_Y
        rospy.loginfo(f"🚗 导航前往: ({tx:.2f}, {ty:.2f}) | 模式: {'视觉对准' if use_vision else '纯盲跑'}")
        
        try:
            goal = Pose2D()
            goal.x = dx
            goal.y = dy
            goal.theta = 0.0 
            # 开启避障
            self.srv_move_base(goal, "odom", True, False)
            
            # 盲跑等待时间
            time.sleep(10) 
            
            # 如果启用视觉，则进行精调
            if use_vision:
                self.align_tag(tag_id)
                
        except Exception as e:
            rospy.logerr(f"导航失败: {e}")

    def move_retreat(self):
        """倒车 (0.6米)"""
        rospy.loginfo("🔙 正在倒车...")
        try:
            goal = Pose2D()
            goal.x = -0.6
            goal.y = 0.0
            goal.theta = 0.0
            self.srv_move_base(goal, "base_link", False, False)
            time.sleep(5)
        except Exception as e:
            rospy.logerr(f"倒车失败: {e}")

    def align_tag(self, tag_id):
        """视觉对准 (仅任务二使用)"""
        rospy.loginfo(f"👀 正在寻找 ID-{tag_id} 进行对准...")
        try:
            self.srv_align(ar_id=tag_id, goal_dist=0.35)
            time.sleep(3)
        except Exception as e:
            rospy.logwarn(f"对准异常: {e}")

    def arm_pose(self, x, y, z, wait_time=2.5):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        self.srv_arm_move(pose=p)
        time.sleep(wait_time) 

    # --- 任务逻辑 ---

    def run_task_1(self):
        rospy.loginfo("\n========== [任务一] 纯盲跑上料 ==========")
        
        # 1. 盲跑去 5号台 (不进行视觉对准，防止车身歪)
        self.move_global(STATION_5_X, STATION_5_Y, use_vision=False)
        
        # 2. 机械臂动作
        rospy.loginfo("🤖 机械臂动作：Buffer(后) -> 台面(前)")
        
        # A. 复位到中转点
        self.arm_pose(HOME_X, HOME_Y, HOME_Z, 2.0)
        
        # B. 去 Buffer 抓取 (使用修正后的负X坐标)
        rospy.loginfo(f"   >>> 移动到 Buffer: X={BUFFER_X}, Y={BUFFER_Y}")
        self.arm_pose(BUFFER_X, BUFFER_Y, BUFFER_Z_LIFT, 2.0) # 上方
        self.arm_pose(BUFFER_X, BUFFER_Y, BUFFER_Z_GRAB, 2.0) # 下降
        
        # 吸气
        self.srv_pump_on()
        rospy.loginfo("   <吸泵开启>...")
        time.sleep(2.0)
        
        # 抬起
        self.arm_pose(BUFFER_X, BUFFER_Y, BUFFER_Z_LIFT, 3.0)
        
        # C. 回到中转点 (防止撞车)
        self.arm_pose(HOME_X, HOME_Y, HOME_Z, 3.0)
        
        # D. 放置到台面
        self.arm_pose(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_HOVER, 3.0)
        self.arm_pose(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_DROP, 2.0)
        
        self.srv_pump_off()
        rospy.loginfo("   <吸泵关闭> 放置完成")
        time.sleep(1.0)
        
        # E. 收回
        self.arm_pose(HOME_X, HOME_Y, HOME_Z, 2.0)
        
        # 3. 撤离
        self.move_retreat() # 倒车
        rospy.loginfo("🏠 任务一完成，斜向返回起点...")
        self.move_global(START_X, START_Y, use_vision=False) # 盲跑回起点

    def run_task_2(self):
        rospy.loginfo("\n========== [任务二] 取料 ==========")
        
        # 1. 去 1号台 (这里建议保留视觉对准，或者如果你想盲跑也可以改成 False)
        # 考虑到取料需要对得很准，建议保留 True
        self.move_global(STATION_1_X, STATION_1_Y, use_vision=True, tag_id=STATION_1_ID)
        
        # 横向微调 (针对右手边框)
        try:
            goal = Pose2D()
            goal.x = 0.0
            goal.y = -0.1 # 向右平移 10cm
            goal.theta = 0.0
            self.srv_move_base(goal, "base_link", False, False)
            time.sleep(2)
        except: pass
        
        # 2. 视觉抓取
        rospy.loginfo("🤖 启动视觉抓取...")
        buffer_target = Pose()
        buffer_target.position.x = BUFFER_X
        buffer_target.position.y = BUFFER_Y
        buffer_target.position.z = BUFFER_Z_GRAB + 40
        
        self.srv_pick_ar(number=9, mode=0, pose=buffer_target)
        time.sleep(10.0)
        
        # 3. 撤离
        self.move_retreat()
        self.move_global(START_X, START_Y, use_vision=False)

    def run(self):
        self.run_task_1()
        rospy.loginfo("⏸️ 中场休息...")
        time.sleep(2)
        self.run_task_2()
        rospy.loginfo("\n🏆 任务结束！")

if __name__ == "__main__":
    cmdr = MissionCommander()
    cmdr.run()