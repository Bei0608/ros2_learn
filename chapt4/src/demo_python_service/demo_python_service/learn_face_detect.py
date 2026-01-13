import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory


def main():
    # 获取图片真实路径
    #get_package_share_directory () 函数用于通过功能包的名字获取与该功能包的安装目录
    defautl_image_path = get_package_share_directory(
        'demo_python_service') + '/resource/default.jpg'
    
    # 使用opencv2读取图片
    image = cv2.imread(defautl_image_path)
    
    # 查找图像中的所有人脸
    # face_recognition.face_locations 函数用于检测图像中的人脸位置
    # 参数说明：
    # image: 输入图像
    # number_of_times_to_upsample: 上采样次数，增加检测精度
    # model: 使用的模型类型，'hog'为基于HOG的模型，'cnn'为基于CNN的模型
    face_locations = face_recognition.face_locations(
        image,number_of_times_to_upsample=1,model='hog')
    
    # 绘制每个人脸的边框
    # cv2.rectangle 函数用于在图像上绘制矩形
    # 参数说明：
    # image: 输入图像
    # (left, top): 矩形左上角坐标
    # (right, bottom): 矩形右下角坐标
    # (0, 0, 255): 矩形颜色，BGR格式，这里为红色
    # 4: 矩形线条粗细
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 4)
    
    # 显示图像
    # cv2.imshow 函数用于显示图像
    cv2.imshow("Face Detected", image)
    
    # cv2.waitKey 函数用于等待键盘输入
    cv2.waitKey(0)
    
    # cv2.destroyAllWindows 函数用于关闭所有OpenCV窗口
    # cv2.destroyAllWindows()
    