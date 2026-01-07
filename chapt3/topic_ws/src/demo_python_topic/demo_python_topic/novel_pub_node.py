import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue() # 创建存放小说的队列、
        # 创建发布者，发布小说
        self.novels_publisher = self.create_publisher(String, 'novels', 10)  # 话题接口 话题名称 服务质量
        # 创建定时器，定时发布小说
        self.timer = self.create_timer(5.0, self.timer_callback)  # 每5秒发布一次小说内容 回调函数
        
        
    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info(f'下载完成 {url}')
        for line in response.text.splitlines():  # 按行分割，放入队列
            self.novels_queue_.put(line)
            
            
    def timer_callback(self):
        if self.novels_queue_.qsize() > 0:  # 队列中有输出，取出发布一行
            msg = String() # 创建消息对象
            msg.data = self.novels_queue_.get()  # 对消息结构体进行赋值
            self.novels_publisher.publish(msg)  # 发布消息
            self.get_logger().info(f'发布小说内容: {msg.data}')  # 打印日志
            
    
def main():
    rclpy.init()
    node = NovelPubNode('novel_publisher')
    node.download_novel('http://localhost:8000/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()