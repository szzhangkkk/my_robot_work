import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AgileMazeSolver(Node):
    
    def __init__(self):
        super().__init__('maze_solver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        # --- 1. 参数回调 (更敏捷) ---
        self.target_dist = 0.60     # 🌟 改回 0.6m，贴近一点才能看清路口
        self.gap_threshold = 1.0    # 🌟 阈值降低到 1.0m，更容易触发拐弯
        
        # --- 2. 安全参数 ---
        self.backup_dist = 0.25     # 救命倒车距离
        self.front_stop_dist = 0.60 # 前方刹车距离
        self.exit_threshold = 3.0   # 出口判定距离
        
        # --- 3. PID 参数 ---
        self.kp = 1.0 # 稍微调高一点灵敏度
        self.kd = 0.3
        self.prev_error = 0.0
        self.deadband = 0.03 # 死区调小一点
        
        # --- 4. 状态变量 ---
        self.cornering_timer = 0    
        self.is_finished = False    
        
        self.get_logger().info("敏捷修正版: 更近的贴墙距离 + 更灵敏的路口检测")

    def listener_callback(self, msg):
        if self.is_finished: return

        # 数据清洗
        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = 10.0
        ranges[ranges == 0.0] = 10.0
        min_global = np.min(ranges)
        
        # 视野分区 (180=前, 90=右)
        front = np.min(ranges[160:200])       
        right_side = np.min(ranges[60:120])   
        right_front = np.min(ranges[120:160]) # 关键！用于辅助判断路口
        left_side = np.min(ranges[240:300])

        cmd = Twist()

        # --- 【阶段 0】: 胜利检测 ---
        if front > self.exit_threshold and left_side > self.exit_threshold and right_side > self.exit_threshold:
            self.get_logger().info(f"🎉 抵达出口！")
            self.stop_robot()
            self.is_finished = True
            return

        # --- 【阶段 1】: 倒车救命 (不变) ---
        if min_global < self.backup_dist:
            # self.get_logger().warn("贴脸了 -> 倒车")
            cmd.linear.x = -0.15
            if np.argmin(ranges) < 180: cmd.angular.z = 0.6 
            else: cmd.angular.z = -0.6
            self.publisher_.publish(cmd)
            return

        # --- 【阶段 2】: 过弯盲开 (优化) ---
        if self.cornering_timer > 0:
            # 安全中断检测
            if front < self.front_stop_dist:
                 self.cornering_timer = 0 # 前面突然有墙，立刻中断，交给下一帧左转
            else:
                self.cornering_timer -= 1
                cmd.linear.x = 0.18 # 稍微快一点通过
                # 🌟 关键小技巧：盲开时给一点点微弱的右转倾向
                # 这样既保证屁股过墙角，又能让车头提前对准新路口
                cmd.angular.z = -0.2 
                self.publisher_.publish(cmd)
                return

        # --- 【阶段 3】: 导航决策 (核心修改) ---
        
        # A. 前方有墙 -> 左转
        if front < self.front_stop_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.7 # 原地左转
            self.prev_error = 0.0 

        # B. 发现路口 -> 启动过弯流程
        # 🌟 修改条件：右边空了(>1.0)，并且右前方也没有立刻挡住(>0.8)
        elif right_side > self.gap_threshold and right_front > 0.8:
            self.get_logger().info(">>> 发现路口，准备右转")
            # 🌟 缩短盲开时间：10帧够了，因为现在离墙近
            self.cornering_timer = 10 
            cmd.linear.x = 0.18
            cmd.angular.z = -0.3 # 启动瞬间给个初速度

        # C. 路口过完后的后续处理
        elif right_side > self.gap_threshold: 
            # 盲开结束了，右边还是空，说明正对路口，加速右转进去
            cmd.linear.x = 0.1
            cmd.angular.z = -0.9 
            self.prev_error = 0.0

        # D. 沿墙巡航 (PID)
        else:
            error = self.target_dist - right_side
            if abs(error) < self.deadband: error = 0.0
            
            # PID
            turn = (error * self.kp) + ((error - self.prev_error) * self.kd)
            self.prev_error = error
            turn = np.clip(turn, -0.8, 0.8) # 放宽一点转速限制
            
            cmd.linear.x = 0.25
            cmd.angular.z = turn

        self.publisher_.publish(cmd)

    def stop_robot(self):
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    solver = AgileMazeSolver()
    try:
        rclpy.spin(solver)
    except KeyboardInterrupt:
        pass
    finally:
        solver.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
