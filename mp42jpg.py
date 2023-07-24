import cv2
from cv2 import VideoCapture
from cv2 import imwrite
import os
import sys
def deletFile(folder,tail):
    filelist=os.listdir(folder)
    for file in filelist:
        if file.endswith(tail):
            del_file =os.path.join(folder, file)  
            os.remove(del_file) 
            print("已经删除：",del_file) 
 
def save_image(image, addr, num):
    address = addr + str(num).zfill(5) + '.jpg'
    img_90 = cv2.flip(cv2.transpose(image), 1)
    imwrite(address, img_90)


    
if __name__ == '__main__':
    video_path = sys.argv[1]  
    time_interval=int(sys.argv[2] )
    video_dir = os.path.dirname(os.path.abspath(video_path)) 
    out_path = os.path.join(video_dir, 'img_')
    print('video_path = ',video_path)
    print('time_interval = ',time_interval)
    print('video_dir = ',video_dir)
    print('out_path = ',out_path)
    deletFile(video_dir,'.jpg')

 
    ######
    #time_interval = 5 #时间间隔
 
    # 读取视频文件
    videoCapture = VideoCapture(video_path)
 
    # 读帧
    success, frame = videoCapture.read()
    print(success)
 
    i = 0
    j = 0 
 
    while success:
        i = i + 1
        if (i % time_interval == 0):
            print('save frame:', i)
            save_image(frame, out_path, j+0)
            j = j + 1
        success, frame = videoCapture.read()