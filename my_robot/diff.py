import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MazeSolver(Node):
    
    def __init__(self):
        super().__init__('maze_solver')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        # 定义参数
        self.target_dist = 0.5    # 离右侧墙壁的目标距离
        self.safe_dist = 0.6      # 前方安全减速距离
        self.stop_dist = 0.4      # 前方强制转弯距离
        self.is_finished = False  # 停止标志

    def listener_callback(self, msg):
        if self.is_finished:
            return

        # 1. 数据预处理（处理inf值，将其转为最大量程）
        ranges = [r if r != float('inf') else msg.range_max for r in msg.ranges]
        
        # 根据你的激光雷达索引定义方位 (假设180度是正前方)
        # 注意：如果你的机器人方向不对，请微调这些索引
        front = min(ranges[160:200])
        right = min(ranges[45:135])
        left  = min(ranges[225:315])
        all_clear = min(ranges) # 用于判断是否离开迷宫

        # 2. 停止逻辑：如果四周都非常空旷（例如距离都大于5米），认为已离开迷宫
        if front > 5.0 and left > 5.0 and right > 5.0:
            self.stop_robot()
            self.get_logger().info("Exit reached! Stopping...")
            self.is_finished = True
            return

        command_msg = Twist()

        # 3. 核心决策逻辑：右手定则
        if front < self.stop_dist:
            # 情况1：前方有墙 -> 原地左转
            command_msg.linear.x = 0.0
            command_msg.angular.z = 1.0 
        elif right > 1.0:
            # 情况2：右侧太远（遇到右转路口） -> 弧线右转找墙
            command_msg.linear.x = 0.15
            command_msg.angular.z = -0.8
        elif right < self.target_dist - 0.1:
            # 情况3：离右墙太近 -> 稍微左偏
            command_msg.linear.x = 0.15
            command_msg.angular.z = 0.3
        elif right > self.target_dist + 0.1:
            # 情况4：离右墙太远 -> 稍微右偏
            command_msg.linear.x = 0.15
            command_msg.angular.z = -0.3
        else:
            # 情况5：保持直线行驶
            command_msg.linear.x = 0.2
            command_msg.angular.z = 0.0

        self.__publisher.publish(command_msg)

    def stop_robot(self):
        stop_msg = Twist()
        self.__publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    solver = MazeSolver()
    try:
        rclpy.spin(solver)
    except KeyboardInterrupt:
        pass
    finally:
        solver.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
