import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import face_recognition
import time


class FaceDetectNode(Node):
    def  __init__(self):
        super().__init__('face_detect_node')
        
        # opencv 和 ROS2 的 Image格式不兼容，需要使用ROS2提供的 CvBridge 进行转换
        # 用于将 opencv 的image格式转换为ROS2的Image格式，反之亦然
        self.bridge = CvBridge()
        
        # 创建一个名为/face_detect的服务，服务类型为FaceDetect
        # 当有请求到达时，调用detect_face_callback函数进行处理
        # create_service(服务类型, 服务名称, 回调函数)
        self.server = self.create_service(FaceDetector, '/face_detect', self.
                                          detect_face_callback)
        self.defaut_image_path = get_package_share_directory('demo_python_service') + \
            '/resource/test1.jpg'
        # 设置人脸检测的参数
        self.upsample_times = 1
        self.model = "hog"
        
        
    def detect_face_callback(self,request,response):
        
        # 判断请求中是否包含图像数据，如果有则使用请求中的图像进行人脸检测
        # 如果没有图像数据，则使用默认图像进行人脸检测
        # 1. 【读 request】：像读取函数参数一样，拿客户端传来的数据
        # "客户端，你传给我的 image 里有数据吗？"         
        if request.image.data:
            # "既然有，那我就拿这个 image 去做转换"
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.defaut_image_path)
            
        start_time = time.time()
        
        self.get_logger().info('加载图像，开始进行人脸检测')
        
        # 使用face_recognition库进行人脸检测
        # face_locations返回一个包含每张人脸位置的列表，每个位置是(top, right, bottom, left)的元组
        # face_locations()函数的参数：
        # cv_image: 要检测的图像，必须是RGB格式
        # number_of_times_to_upsample: 图像上采样的次数，增加这个值可以帮助检测到更小的人脸，但会增加计算时间
        # model: 使用的检测模型，可以是"hog"（速度较快，适合CPU）或"cnn"（精度较高，适合GPU）
        face_locations = face_recognition.face_locations(cv_image, 
                                                         number_of_times_to_upsample=self.
                                                         upsample_times,
                                                         model=self.model)
        end_time = time.time()
        
        self.get_logger().info(f'检测完成，耗时{end_time - start_time}')
         

        # 2. 【写 response】：像填写函数返回值一样，把结果存进去
        # 注意：此时 response 还是空的（或者全是0），你要把结果填进去
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        
        
        # 将每张人脸的位置添加到响应中
        # face_locations中的每个元素是一个包含(top, right, bottom, left)的元组
        # 将这些值分别添加到响应的对应列表中
        # response.top, response.right, response.bottom, response.left都是列表
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        
        # 3. 【返回】：把填好的结果交回给 ROS，ROS 会负责把它传回给客户端
        return response
    
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    rclpy.spin(node) 
    rclpy.shutdown()
            