from cv2 import VideoCapture
from cv2 import imwrite
import os
import sys

def readCamera(colmapCameraTxt):
    with open(colmapCameraTxt, 'r') as f:
        while True:
            line = f.readline()
            if not line:break
            if line.startswith('#'):continue
            components = line.split() 
            if len(components)!=16:continue
            [CAMERA_ID, MODEL, w, h, fl_x, fl_y]

colmapDir =  os.path.normpath ('D:/repo/mvs_mvg_bat/viewerout/colmap')
colmapCameraTxt = os.path.join(colmapDir,'cameras.txt')
readCamera(colmapCameraTxt)

