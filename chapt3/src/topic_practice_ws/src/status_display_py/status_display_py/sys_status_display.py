import rclpy
import threading
from rclpy.node import Node
from status_interfaces.msg import SystemStatus
from PyQt5.QtWidgets import QApplication, QLabel,QVBoxLayout, QWidget
from PyQt5.QtCore import pyqtSignal, QObject

# 在 ROS 2 + PyQt 的混合开发中始终保持 class MyClass(Node, QWidget): 
# 的顺序，并且在 __init__ 里显式地分别初始化它们

"""
    在 Python 中，当你调用 QWidget.__init__(self) 时，
    QWidget 内部会自动调用 super().__init__() 来寻找下一个父类。 
    根据你定义的顺序(QWidget 在前,Node 在后),
    Python 认为 Node 是 QWidget 的父亲。 
    于是 QWidget 尝试调用 Node 的初始化函数，
    但它没有传任何参数（因为它不知道还要传 node_name),
    导致了 TypeError: missing 1 required positional argument
    所以要注意 Node在前,QWidget在后
"""

class SysStatusDisplay(Node, QWidget):  
    # 定义信号
    update_signal = pyqtSignal(str)
    # 在 Python 中，信号必须定义为类属性（写在 __init__ 外面）
    
    def __init__(self):
        # 初始化父类
        Node.__init__(self, 'sys_status_display')                   
        QWidget.__init__(self)

        # --- 1. 界面初始化  ---
        self.setWindowTitle('系统状态显示')
        self.resize(400, 300)
        
        # 使用布局管理器让界面更规整
        self.layout = QVBoxLayout()
        self.label = QLabel('等待系统状态消息...')
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)
        
        # --- 2. 信号与槽连接 ---
        # 意思：当 update_signal 触发时，自动调用 self.update_label 函数
        self.update_signal.connect(self.update_label)
        
        # --- 3. 创建订阅者 ---
        self.subscription   = self.create_subscription(
            SystemStatus,           # 参数 1: 消息类型 (类名)
            'system_status',        # 参数 2: 话题名
            self.listener_callback,   # 参数 3: 回调函数 (注意这里不加括号！)
            10)                     # 参数 4: QoS (队列长度)
        
     # --- 4. ROS 回调函数 (运行在子线程) ---   
    def listener_callback(self, msg):
        text = f"""
================ 系统状态 (Py) ================
数据时间: \t{msg.stamp.sec} s
主机名:   \t{msg.host_name}
CPU 使用: \t{msg.cpu_percent} %
内存使用: \t{msg.memory_percent} %
内存总计: \t{msg.memory_total:.2f} MB
内存剩余: \t{msg.memory_available:.2f} MB
网络发送: \t{msg.net_sent:.2f} MB
网络接收: \t{msg.net_recv:.2f} MB
=============================================
        """
        
        # 发射信号，把数据传给主线程
        self.update_signal.emit(text)
        
    # --- 5. Qt 槽函数 (运行在主线程) ---
    def update_label(self, text):
        self.label.setText(text)
        
        
def main(args=None):
    # 1. 初始化 ROS 和 Qt
    rclpy.init(args=args)
    app = QApplication([])

    # 2. 创建节点
    node = SysStatusDisplay()
    node.show()  # 窗口显示
    
    # 3. 多线程处理 
    # 必须开一个新线程去跑 ROS 的 spin，否则 Qt 的界面会卡死
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    
    # 4. 运行 Qt 主循环 
    # 这里的 exec_ 带下划线，是因为 exec 是 Python 关键字
    app.exec_()
    
    # 5. 退出清理
    rclpy.shutdown()
    thread.join()