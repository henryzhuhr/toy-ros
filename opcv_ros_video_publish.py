import cv2 # 导入 OpenCV 库
import rclpy # 导入 rcly 库



#在终端建立一个新包 name = opencv_ros_video_stream
#ros2 pkg create --build-type ament_python(这个什么意思) video_stream
#cd video_stream 进入包目录下开发
#

 #用ros2 pkg list 查看已经安装的包
 #用sudo apt-get install ros-<your_ros2_distro>-cv-bridge 来安装包
from rclpy.node import Node  # 导入 Node 类 生成节点       
from sensor_msgs.msg import Image # 导入 Image 类 生成消息
from cv_bridge import CvBridge # 导入 CvBridge 类 用于图像转换

#新建一个节点，并用来发布视频

#如何建立节点
#如何利用bridge将opencv的图像转换成ros2的图像消息
#如何发布图像消息


#如何订阅发布图像的节点
#如何如何利用bridge将ros2的图像转换成opencv的图像消息
#如何显示图像消息

