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
import cv2 
from threading import Thread
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
def IGL_RAY_TRI_DOT(v1,v2):
    return (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
def IGL_RAY_TRI_SUB(v1,v2):
    dest=[0,0,0]
    dest[0]=v1[0]-v2[0] 
    dest[1]=v1[1]-v2[1] 
    dest[2]=v1[2]-v2[2] 
    return dest
def IGL_RAY_TRI_CROSS(v1,v2):
    dest=[0,0,0]
    dest[0]=v1[1]*v2[2]-v1[2]*v2[1]
    dest[1]=v1[2]*v2[0]-v1[0]*v2[2]
    dest[2]=v1[0]*v2[1]-v1[1]*v2[0]
    return dest    
def intersect_triangle1(s,dir,v0,v1,v2):
    edge1 = IGL_RAY_TRI_SUB(v1,v0)
    edge2 = IGL_RAY_TRI_SUB(v2,v0)
    pvec = IGL_RAY_TRI_CROSS(dir, edge2)
    det = IGL_RAY_TRI_DOT(edge1, pvec)
    if det > 1e-8:
        tvec = IGL_RAY_TRI_SUB(s, v0)
        u = IGL_RAY_TRI_DOT(tvec, pvec);
        if (u < 0.0 or u > det): return 0
        qvec = IGL_RAY_TRI_CROSS( tvec, edge1);
        v = IGL_RAY_TRI_DOT(dir, qvec)
        if (v < 0.0 or u+v > det): return 0
    elif det < -1e-8:
        tvec = IGL_RAY_TRI_SUB(s, v0)
        u = IGL_RAY_TRI_DOT(tvec, pvec)
        if (u < 0.0 or u > det): return 0
        qvec = IGL_RAY_TRI_CROSS(tvec, edge1)
        v = IGL_RAY_TRI_DOT(dir, qvec) 
        if (v < 0.0 or u+v > det): return 0
    else:return 0
    return 1
def ray_mesh_intersect(s,dir,V,F):
    hitCnt=0
    for f in F:
        v0 = V[f[0]]
        v1 = V[f[1]]
        v2 = V[f[2]]
        hit = intersect_triangle1(s,dir,v0,v1,v2)
        if hit>0:hitCnt+=1
    return hitCnt
 
def rays_mesh_intersect(s,dirs,V,F):
    hitCnt=[]
    for d in dirs: 
        hitCnt.append(ray_mesh_intersect(s,d,V,F))
    return hitCnt
class MyThread(Thread):
    def __init__(self, func, args): 
        Thread.__init__(self)
        self.func = func
        self.args = args
        self.result = None

    def run(self):
        self.result = self.func(*self.args)

    def getResult(self):
        return self.result
def rays_mesh_intersect2(s,dirs,V,F):
    hitCnt=np.empty([len(dirs)],dtype=int)
    threads=[]
    for i in range(len(dirs)):
        t1 = MyThread(ray_mesh_intersect, (s,dirs[i],V,F))
        t1.start() 
        threads.append(t1)
    for i in range(len(dirs)): 
        t1.join() 
        hitCnt[i] = t1.getResult() 
    return hitCnt
def landmark2d(img,detection_result):
    face_landmarks_list = detection_result.face_landmarks
    if len(face_landmarks_list)!=1:
        print("len(face_landmarks_list)!=1")
        return None
    face_landmarks = face_landmarks_list[0]
    landmark3dList=np.zeros((len(face_landmarks), 3)) 
    for i in range(len(face_landmarks)):
        landmark3dList[i]=[face_landmarks[i].x, face_landmarks[i].y, face_landmarks[i].z] 
    norm=np.linalg.norm(landmark3dList, ord=2,axis=1)
    dir = landmark3dList/norm.reshape((-1,1))
    
    hits = rays_mesh_intersect2([0,0,0],dir,landmark3dList,constFaces) 
     
    landmark2dNorm = landmark3dList[:,0:2]
    xy=np.array([[img.shape[1]-1,img.shape[0]-1]])
    landmark2d=landmark2dNorm*xy
    
    data=[]
    for i in range(landmark2d.shape[0]):
        if hits[i]>1:data.append((-1,-1))
        elif landmark2dNorm[i,0]>0.95 or landmark2dNorm[i,0]<0.05 or landmark2dNorm[i,1]>=0.95 or landmark2dNorm[i,1]<0.05  :          data.append((-1,-1))
        else :data.append((landmark2d[i,0],landmark2d[i,1]))
    return data

def draw_landmarks_on_image(rgb_image, detection_result):
  face_landmarks_list = detection_result.face_landmarks
  annotated_image = np.copy(rgb_image)
  # Loop through the detected faces to visualize.
  for idx in range(len(face_landmarks_list)):
    face_landmarks = face_landmarks_list[idx]

    # Draw the face landmarks.
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
#    solutions.drawing_utils.draw_landmarks(
#        image=annotated_image,
#        landmark_list=face_landmarks_proto,
#        connections=mp.solutions.face_mesh.FACEMESH_CONTOURS,
#        landmark_drawing_spec=None,
#        connection_drawing_spec=mp.solutions.drawing_styles
#        .get_default_face_mesh_contours_style())
#    solutions.drawing_utils.draw_landmarks(
#        image=annotated_image,
#        landmark_list=face_landmarks_proto,
#        connections=mp.solutions.face_mesh.FACEMESH_IRISES,
#          landmark_drawing_spec=None,
#          connection_drawing_spec=mp.solutions.drawing_styles
#          .get_default_face_mesh_iris_connections_style())

  return annotated_image

def plot_face_blendshapes_bar_graph(face_blendshapes):
  # Extract the face blendshapes category names and scores.
  face_blendshapes_names = [face_blendshapes_category.category_name for face_blendshapes_category in face_blendshapes]
  face_blendshapes_scores = [face_blendshapes_category.score for face_blendshapes_category in face_blendshapes]
  # The blendshapes are ordered in decreasing score value.
  face_blendshapes_ranks = range(len(face_blendshapes_names))

  fig, ax = plt.subplots(figsize=(12, 12))
  bar = ax.barh(face_blendshapes_ranks, face_blendshapes_scores, label=[str(x) for x in face_blendshapes_ranks])
  ax.set_yticks(face_blendshapes_ranks, face_blendshapes_names)
  ax.invert_yaxis()

  # Label each bar with values
  for score, patch in zip(face_blendshapes_scores, bar.patches):
    plt.text(patch.get_x() + patch.get_width(), patch.get_y(), f"{score:.4f}", va="top")

  ax.set_xlabel('Score')
  ax.set_title("Face Blendshapes")
  plt.tight_layout()
  plt.show()
def getLandmarks(imgPath): 
    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)
    image = mp.Image.create_from_file(imgPath)
    detection_result = detector.detect(image)
    return landmark2d(image.numpy_view(),detection_result)
def detectSigle():
    imgPath='d:/repo/mvs_mvg_bat/viewer/img_0.jpg'   
    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)

    image = mp.Image.create_from_file(imgPath) 
    detection_result = detector.detect(image) 
    annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
    annotated_image = cv2.cvtColor(annotated_image,  cv2.COLOR_BGR2RGB)
    print(annotated_image.shape)
    frontLandmarks = landmark2d(image.numpy_view(),detection_result)
    print(frontLandmarks)    
    img=cv2.imread(imgPath)
    for p in frontLandmarks:
        if(p[0]<0):continue
        center = (int(p[0]),int(p[1]))
        img = cv2.circle(img, center, 3, [255,255,255], -1) 
    cv2.imshow('result', img)
    cv2.waitKey( ) 
    return frontLandmarks

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        return (fs)
if __name__ == '__main__':    
    #detectSigle()
    #exit(0) 
    imgsRoot=sys.argv[1] 
    jsonRoot=sys.argv[2]   
    imgNames=findAllFile(imgsRoot)
    print(imgNames)  
    imgsPath=[]
    jsonsPath=[]
    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)
    index_=0
    for imgName in imgNames:
        imgPath = os.path.join(imgsRoot,imgName)
        if not imgPath.endswith('.jpg'):continue
        jsonPath = os.path.join(jsonRoot,imgName)+'.json'
        showPath = os.path.join(jsonRoot,imgName)+'.jpg'
        print(imgPath,index_,'/',len(imgNames))
        image = mp.Image.create_from_file(imgPath)
        detection_result = detector.detect(image)
        annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
        annotated_image = cv2.cvtColor(annotated_image,  cv2.COLOR_BGR2RGB)
        frontLandmarks = landmark2d(image.numpy_view(),detection_result)  

        if not frontLandmarks is None:
            img=cv2.imread(imgPath)
            for p in frontLandmarks:
                if(p[0]<0):continue
                center = (int(p[0]),int(p[1]))
                img = cv2.circle(img, center, 3, [255,255,255], -1) 
            cv2.imwrite(showPath,img)
            #cv2.imwrite(showPath,annotated_image)
            data = {'landMarks':frontLandmarks} 
            with open(jsonPath, 'w') as f:
                json.dump(data, f)