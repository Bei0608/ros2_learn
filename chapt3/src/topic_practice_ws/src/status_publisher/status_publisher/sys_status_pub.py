import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # 创建发布者，发布系统状态
        self.status_publisher_ = self.create_publisher(
            SystemStatus, 
            'system_status', 
            10)
        # 创建定时器，定时发布系统状态
        self.timer_ = self.create_timer(2.0, self.timer_callback)  # 每2秒发布一次系统状态 回调函数
        
        
    def timer_callback(self):
        cup_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io = psutil.net_io_counters()
        
        msg = SystemStatus()
        # 获取当前时间戳
        msg.stamp = self.get_clock().now().to_msg()
        # 获取主机名
        msg.host_name = platform.node()
        # 获取CPU使用率百分比
        msg.cpu_percent = cup_percent
        # 获取内存使用情况
        msg.memory_percent = memory_info.percent
        # 获取内存总量和可用量
        msg.memory_total = memory_info.total / (1024 * 1024)  # 转换为MB
        # 获取内存剩余量
        msg.memory_available = memory_info.available / (1024 * 1024)  # 转换为MB
        # 获取网络发送和接收数据总量
        msg.net_sent = net_io.bytes_sent / (1024 * 1024)  # 转换为MB
        msg.net_recv = net_io.bytes_recv / (1024 * 1024)  # 转换为MB
        
        self.get_logger().info(f'发布系统状态: Host={msg.host_name}, CPU={msg.cpu_percent}%, memory_percent={msg.memory_percent}%, memory_total={msg.memory_total}MB, memory_available={msg.memory_available}MB, net_sent={msg.net_sent}MB, net_recv={msg.net_recv}MB')
        self.status_publisher_.publish(msg)
        
        
def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()
        