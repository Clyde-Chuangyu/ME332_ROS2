#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys
import time

# 尝试导入服务
try:
    from linkattacher_msgs.srv import AttachLink, DetachLink
except ImportError:
    try:
        from link_attacher_msgs.srv import AttachLink, DetachLink
    except:
        print("⚠️ 无法导入 linkattacher_msgs")
        sys.exit(1)

# ================= 你的原始参数 (完全未动) =================
GRASP_ANGLES = [0.0, 31.0, 78.0, 70.0]
CARRY_ANGLES = [0.0, -10.0, 20.0, 20.0]
GRIPPER_CLOSE = [-0.009, 0.009] 
GRIPPER_OPEN = [0.0, 0.0]
# =======================================================

class SimpleCarry(Node):
    def __init__(self):
        super().__init__('simple_carry')
        
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # 连接吸附服务
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        print("正在连接控制器...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            print("⚠️  警告: 未找到吸附服务 /ATTACHLINK")
        else:
            print("✅ 吸附服务已连接")

    def move_arm(self, angles_deg, duration=3.0):
        rads = [math.radians(a) for a in angles_deg]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']
        point = JointTrajectoryPoint()
        point.positions = rads
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        goal.trajectory.points = [point]
        self._send_goal(self.arm_client, goal)

    def move_gripper(self, left, right):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['finger_joint_1', 'finger_joint_2']
        point = JointTrajectoryPoint()
        point.positions = [float(left), float(right)]
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points = [point]
        self._send_goal(self.gripper_client, goal)

    def _send_goal(self, client, goal):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.accepted:
            res_future = res.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)

    # --- 关键修改: 使用 arm_link_3 代替 gripper_base_link ---
    def lock_object(self):
        if not self.attach_client.service_is_ready(): return
        
        req = AttachLink.Request()
        req.model1_name = 'my_mobile_manipulator'
        # [修改] 因为固定关节合并，必须用父Link的名字
        req.link1_name  = 'arm_link_3'  
        req.model2_name = 'target_cube'
        req.link2_name  = 'link'
        
        print(f">>> 🔒 正在吸附 {req.model2_name} 到 {req.link1_name} ...")
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # 打印详细结果
        res = future.result()
        if res.success:
            print("✅✅✅ 吸附成功！")
        else:
            print(f"❌❌❌ 吸附失败: {res.message}")

    def unlock_object(self):
        if not self.detach_client.service_is_ready(): return
        req = DetachLink.Request()
        req.model1_name = 'my_mobile_manipulator'
        # [修改] 解锁时也要用同样的名字
        req.link1_name  = 'arm_link_3' 
        req.model2_name = 'target_cube'
        req.link2_name  = 'link'
        
        print(">>> 🔓 正在解锁...")
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    bot = SimpleCarry()

    try:
        print("步骤1: 张开夹爪")
        bot.move_gripper(GRIPPER_OPEN[0], GRIPPER_OPEN[1])

        print("步骤2: 移动到抓取位置")
        bot.move_arm(GRASP_ANGLES)
        time.sleep(0.5)

        print("步骤3: 闭合夹爪")
        bot.move_gripper(GRIPPER_CLOSE[0], GRIPPER_CLOSE[1])
        time.sleep(1.0) 
        
        # 吸附
        bot.lock_object()

        print("步骤4: 抬起手臂")
        bot.move_arm(CARRY_ANGLES)

        print("--------------------------------")
        print("✅ 已抓取。请控制小车移动到目的地。")
        print("到达后按【回车键】放下物体...")
        print("--------------------------------")
        input()

        print("步骤5: 下降手臂")
        bot.move_arm(GRASP_ANGLES, duration=4.0) 
        time.sleep(0.5)

        # 解锁
        bot.unlock_object()

        print("步骤6: 松开夹爪")
        bot.move_gripper(GRIPPER_OPEN[0], GRIPPER_OPEN[1])
        time.sleep(1.0)

        print("步骤7: 手臂复位")
        bot.move_arm([0.0, 0.0, 0.0, 0.0], duration=2.0)
        print("任务完成")

    except KeyboardInterrupt:
        pass
    finally:
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
