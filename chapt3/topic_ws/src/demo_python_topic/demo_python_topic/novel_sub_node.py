import rclpy
from rclpy.node import Node
import threading
from example_interfaces.msg import String
from queue import Queue
import time
import espeakng

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue()  # 创建存放小说的队列
        # 创建订阅者，订阅小说
        self.novels_subscriber_ = self.create_subscription(
            String,
            'novels',
            self.novel_callback,
            10)  # 话题接口 话题名称 回调函数 服务质量
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start()
        
        
    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)  # 将接收到的小说内容放入队列
        
    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:  # 队列中有小说内容
                text = self.novels_queue_.get()  # 取出小说内容
                self.get_logger().info(f'朗读小说内容: {text}')  # 打印日志
                speaker.say(text)  # 朗读小说内容
            else:
                time.sleep(0.1)  # 队列为空，休眠一会儿
                
                
def main(args = None):  
    rclpy.init(args=args)  
    node = NovelSubNode('novel_subscriber')
    rclpy.spin(node)
    rclpy.shutdown()
        


