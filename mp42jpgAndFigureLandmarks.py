import cv2
from cv2 import VideoCapture
from cv2 import imwrite
import json
import os
import sys
import math
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from mediapipe.python.solutions.face_mesh_connections import FACEMESH_TESSELATION
import numpy as np
import matplotlib.pyplot as plt
def deletFile(folder,tail):
    filelist=os.listdir(folder)
    for file in filelist:
        if file.endswith(tail):
            del_file =os.path.join(folder, file)  
            os.remove(del_file) 
            print("已经删除：",del_file) 
def generTriplet(edges): 
    faces=[]
    facesS=[]
    nodeMap={}
    for edge in edges:
        if not edge[0] in nodeMap: nodeMap[edge[0]]=[]
        if not edge[1] in nodeMap: nodeMap[edge[1]]=[]
        nodeMap[edge[0]].append(edge[1])
        nodeMap[edge[1]].append(edge[0])
    for edge in edges:
        a = edge[0]
        b = edge[1]
        cs=list(set(nodeMap[a]) & set(nodeMap[b]))
        if len(cs)>2 or len(cs)<1:
            print('len(cs)>2 or len(cs)',len(cs))
            continue 
        for c in cs:
            tr1=sorted([a,b,c])
            tr1s = str(tr1[0])+' '+str(tr1[1])+' '+str(tr1[2])
            if not tr1s in facesS:
                facesS.append(tr1s)
                faces.append(tr1)
    return faces
constFaces = generTriplet(FACEMESH_TESSELATION)
def ndArrayToList(data):
    r,c=data.shape
    outdata=[]
    if (c==2):  
         for i in range(r): outdata.append((data[i,0],data[i,1]))
    if (c==3):  
         for i in range(r): outdata.append((data[i,0],data[i,1],data[i,2]))
    return outdata
def save_image(image, addr, num):
    address = addr + str(num).zfill(5) + '.jpg'
    #img_90 = cv2.flip(cv2.transpose(image), 1)
    #imwrite(address, img_90)
    imwrite(address, image)
def landmark2d(img,detection_result):
    face_landmarks_list = detection_result.face_landmarks
    if len(face_landmarks_list)!=1:
        #print("len(face_landmarks_list)!=1")
        return None,None
    face_landmarks = face_landmarks_list[0]
    landmark3dList=np.zeros((len(face_landmarks), 3)) 
    for i in range(len(face_landmarks)):
        landmark3dList[i]=[face_landmarks[i].x, face_landmarks[i].y, face_landmarks[i].z] 
 
    xyz=np.array([[img.shape[1],img.shape[0],img.shape[1]]])
    landmark3d=landmark3dList*xyz
    return ndArrayToList(landmark3d),constFaces
    
def draw_landmarks_on_image(rgb_image, detection_result):
  face_landmarks_list = detection_result.face_landmarks
  annotated_image = np.copy(rgb_image)
  # Loop through the detected faces to visualize.
  for idx in range(len(face_landmarks_list)):
    face_landmarks = face_landmarks_list[idx] 
    face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    face_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
    ]) 
    solutions.drawing_utils.draw_landmarks(
        image=annotated_image,
        landmark_list=face_landmarks_proto,
        connections=mp.solutions.face_mesh.FACEMESH_TESSELATION,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp.solutions.drawing_styles
        .get_default_face_mesh_tesselation_style()) 
  return annotated_image
    
if __name__ == '__main__':
    video_path = sys.argv[1]  
    time_interval=int(sys.argv[2] )
    jsonRoot=sys.argv[3] 
    video_dir = os.path.dirname(os.path.abspath(video_path)) 
    out_path = os.path.join(video_dir, 'img_')
    print('video_path = ',video_path)
    print('time_interval = ',time_interval)
    print('video_dir = ',video_dir)
    print('out_path = ',out_path)
    deletFile(video_dir,'.jpg')

    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)
 
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
            #frame = cv2.cvtColor(frame,  cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            detection_result = detector.detect(image)
            annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
            annotated_image = cv2.cvtColor(annotated_image,  cv2.COLOR_BGR2RGB)
            frontLandmarks3d,faces = landmark2d(image.numpy_view(),detection_result)  
            if not frontLandmarks3d is None:         
                jsonPath = os.path.join(jsonRoot,'img_'+str(j).zfill(5))+'.json'
                showPath = os.path.join(jsonRoot,'img_'+str(j).zfill(5))+'.jpg'
                #print(imgPath,index_,'/',len(imgNames),' (',len(frontLandmarks3d))
                data = {'imgHeight':image.height,'imgWidth':image.width,'frontLandmarks3d':frontLandmarks3d,'faces':faces}
                with open(jsonPath, 'w') as f:
                    json.dump(data, f)
                annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
                cv2.imwrite(showPath, annotated_image)
            else:
                #print(imgPath,index_,'/',len(imgNames),' (0')
                continue
            save_image(frame, out_path, j+0)
            j = j + 1
        success, frame = videoCapture.read()