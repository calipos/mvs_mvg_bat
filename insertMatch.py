import struct
import os
from cv2 import VideoCapture
from cv2 import imwrite
import json
def readSfmJson(path):
    with open(path) as f:
        data = json.load(f)
        views = data['views']
    return views
def readAdditionalMatch(path):
    with open(path, 'r', encoding='utf-8') as f:
        for ann in f.readlines():
            if ann.startswith( '#' ): continue
            segments = ann.split()
            if len(segments)%3!=1:continue
            pointName = segments[0]
            imgSize = int(len(segments)/3)
            imgNames=[]
            xys=[]
            for i in range(imgSize):
                imgNames.append(segments[1+3*i])
                xys.append([float(segments[2+3*i]), float(segments[3+3*i])])
            #print(imgNames)
            #print(xys)
def readPutativeMatch(path):
    with open(path, 'r') as f:
        while True:
            line = f.readline()
            if not line:
               break
            pair = ann.split()
            if len(pair)!=2:return None
            I=int(pair[0])
            J=int(pair[1])
            pairCnt = int(f.readline())
            
            for i in range(pairCnt):
                matchline = f.readline()
if __name__ == '__main__':
 
    additionalMatchPath='E:/viewer/prior.txt'
    sfmJsonPath='E:/viewerout/sfm/matches/sfm_data.json'
    putativeMatchPath='E:/viewerout/sfm/matches/matches.putative.txt'
    readSfmJson(sfmJsonPath) 
    readAdditionalMatch(additionalMatchPath)
    readPutativeMatch(putativeMatchPath)