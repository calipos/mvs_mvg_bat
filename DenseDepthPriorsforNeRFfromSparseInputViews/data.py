import struct
import os
import numpy as np
import json
import random
import cv2
def readData(path):
    f=open(path,"rb")
    data = f.read(164)
    headData = struct.unpack("ddddiidddddddddddddddi", data)
    fx = headData[0]
    fy = headData[1]
    cx = headData[2]
    cy = headData[3]
    height = headData[4]
    width = headData[5]  
    idx=0
    worldToCamera = np.zeros([12])
    for r in range(12): 
            worldToCamera[r]=headData[6+idx]
            idx+=1
    worldToCamera=worldToCamera.reshape([3,4])
    cameraT= np.zeros([3])
    cameraT[0] = headData[18]
    cameraT[1] = headData[19]
    cameraT[2] = headData[20]
    rayCnt = headData[21]

    xyzs=np.zeros([3*rayCnt])
    dirs=np.zeros([3*rayCnt])
    rgbs=np.zeros([3*rayCnt])
    dists=np.zeros([rayCnt])
    sigmas=np.zeros([rayCnt])
    idx=0
    for i in range(rayCnt): 
        data = f.read(88)
        raydat = struct.unpack("ddddddddddd", data)
        xyzs[3*i]=raydat[0]
        xyzs[3*i+1]=raydat[1]
        xyzs[3*i+2]=raydat[2]
        dirs[3*i]=raydat[3]
        dirs[3*i+1]=raydat[4]
        dirs[3*i+2]=raydat[5]
        rgbs[3*i]=raydat[6]
        rgbs[3*i+1]=raydat[7]
        rgbs[3*i+2]=raydat[8]
        dists[i]=raydat[9]
        sigmas[i]=raydat[10]
    
    xyzs=xyzs.reshape([rayCnt,3])
    dirs=dirs.reshape([rayCnt,3])
    rgbs=rgbs.reshape([rayCnt,3])
    return fx,fy,cx,cy,height,width,worldToCamera,cameraT,xyzs,dirs,rgbs,dists,sigmas

def generateDataJson(basedir):
    print('basedir=',basedir)
    trainPercent=0.8
    valPercent=0.2
    testPercent=0.2
    binFiles=[]
    for root,folders,files in os.walk(basedir):
        for file in files:
            if(file.endswith('.bin')):
                binFiles.append(file)
    idxRand=np.arange(len(binFiles))
    np.random.shuffle (idxRand )
    #print(idxRand)
    trainEnd = int(trainPercent*len(binFiles))
    trainIdx=np.arange(0,trainEnd)
    testIdx=np.arange(trainEnd,len(binFiles))
    valIdx=np.arange(trainEnd,len(binFiles))
    trainData = list(np.array(binFiles)[idxRand[trainIdx]])
    testData = list(np.array(binFiles)[idxRand[testIdx]])
    valData = list(np.array(binFiles)[idxRand[valIdx]]) 
    with open(os.path.join(basedir,'trainData.json'),"w") as f:
        json.dump(trainData,f)
    with open(os.path.join(basedir,'testData.json'),"w") as f:
        json.dump(testData,f)
    with open(os.path.join(basedir,'valData.json'),"w") as f:
        json.dump(valData,f)
    return trainData,testData,valData
    # with open(os.path.join(basedir,'trainData.json'),'r') as load_f:
    #     trainData = json.load(load_f)
    # with open(os.path.join(basedir,'testData.json'),'r') as load_f:
    #     testData = json.load(load_f)
    # with open(os.path.join(basedir,'valData.json'),'r') as load_f:
    #     valData = json.load(load_f)

class ImgRays:
    def __init__(self,worldToCamera,cameraT,xyzs,dirs,rgbs,dists,sigmas):
        self.worldToCamera = worldToCamera
        self.cameraT = cameraT
        self.xyzs = xyzs
        self.dirs = dirs
        self.rgbs = rgbs
        self.dists = dists
        self.sigmas = sigmas
     

def readTotalData(basedir,fileList):
    dataCnt = len(fileList)
    ImgRayss=[]
    for i in range(dataCnt):
        fx,fy,cx,cy,height,width,worldToCamera,cameraT,xyzs,dirs,rgbs,dists,sigmas= readData(os.path.join(basedir,fileList[i]))
        ImgRayss.append(ImgRays(worldToCamera,cameraT,xyzs,dirs,rgbs,dists,sigmas)) 
    return fx,fy,cx,cy,height,width,ImgRayss