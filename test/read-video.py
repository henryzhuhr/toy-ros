"""
使用 OpenCV 读取视频 demo
"""
import cv2

def main():
  
  VIDEO_PATH='.cache/sky_train.mp4'
  capture = cv2.VideoCapture(VIDEO_PATH)  # 打开视频文件

  if not capture.isOpened():
    print("Error: Could not open video.")  # 检查视频是否成功打开
    exit() 

  while True:  # 逐帧显示视频
    ret, frame = capture.read()  #ret 为 True 时表示成功读取帧，frame 为读取到的帧
    if not ret:
      break
    cv2.imshow("Frame", frame)  # 显示帧
    if cv2.waitKey(1) & 0xFF == ord('q'):  # & 0xFF：这是一个位运算，用于将返回值与
                                           # 0xFF 进行按位与操作，确保只保留最低的 8 位，
                                           #这是为了兼容某些系统上的差异。
      break

  capture.release()
  cv2.destroyAllWindows()  # 释放资源

if __name__ == "__main__":
    main()