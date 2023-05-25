import json
import os
import sys
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import matplotlib.pyplot as plt
import cv2 
def landmark2d(img,detection_result):
    face_landmarks_list = detection_result.face_landmarks
    if len(face_landmarks_list)!=1:
        print("len(face_landmarks_list)!=1")
        return None
    face_landmarks = face_landmarks_list[0]
    landmark3dList=[]
    for landmark in face_landmarks:
        landmark3dList.append([landmark.x, landmark.y, landmark.z])
        #print(landmark.x, landmark.y, landmark.z)
    landmark3d = np.asarray(landmark3dList)
    landmark2dNorm = landmark3d[:,0:2]
    xy=np.array([[img.shape[1]-1,img.shape[0]-1]])
    landmark2d=landmark2dNorm*xy
    
    data=[]
    for i in range(landmark2d.shape[0]):
        data.append((landmark2d[i,0],landmark2d[i,1]))
    return data

def draw_landmarks_on_image(rgb_image, detection_result):
  face_landmarks_list = detection_result.face_landmarks
  annotated_image = np.copy(rgb_image)
  print(len(face_landmarks_list))
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
    solutions.drawing_utils.draw_landmarks(
        image=annotated_image,
        landmark_list=face_landmarks_proto,
        connections=mp.solutions.face_mesh.FACEMESH_CONTOURS,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp.solutions.drawing_styles
        .get_default_face_mesh_contours_style())
    solutions.drawing_utils.draw_landmarks(
        image=annotated_image,
        landmark_list=face_landmarks_proto,
        connections=mp.solutions.face_mesh.FACEMESH_IRISES,
          landmark_drawing_spec=None,
          connection_drawing_spec=mp.solutions.drawing_styles
          .get_default_face_mesh_iris_connections_style())

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
    imgPath='E:/viewer/im_3.jpg'   
    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)

    image = mp.Image.create_from_file(imgPath)

    # STEP 4: Detect face landmarks from the input image.
    detection_result = detector.detect(image)
    #print(detection_result)
    # STEP 5: Process the detection result. In this case, visualize it.
    annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
    annotated_image = cv2.cvtColor(annotated_image,  cv2.COLOR_BGR2RGB)
    print(annotated_image.shape)
    return landmark2d(image.numpy_view(),detection_result)
    #cv2.imshow('result', annotated_image)
    #cv2.waitKey( ) 
if __name__ == '__main__': 
    
    imagePathSetPath=sys.argv[1]  
    f= open(imagePathSetPath,'r') 
    imgsPath=[]
    jsonsPath=[]
    base_options = python.BaseOptions(model_asset_path='face_landmarker_v2_with_blendshapes.task')
    options = vision.FaceLandmarkerOptions(base_options=base_options,
                                           output_face_blendshapes=True,
                                           output_facial_transformation_matrixes=True,
                                           num_faces=1)
    detector = vision.FaceLandmarker.create_from_options(options)
    i=0
    for line in f:
        if i%2==0:  imgsPath.append(line.strip())
        else : jsonsPath.append(line.strip())
        i+=1
    print(jsonsPath)
    if(len(imgsPath)!=len(jsonsPath)):
        print('len()!=len()')
        exit(0)
    for idx in range(len(imgsPath)):
        image = mp.Image.create_from_file(imgsPath[idx])
        print(imgsPath[idx])
        detection_result = detector.detect(image)
        lms = landmark2d(image.numpy_view(),detection_result) 
        if not lms is None:
            data = {'landMarks':lms} 
            with open(jsonsPath[idx], 'w') as f:
                json.dump(data, f)