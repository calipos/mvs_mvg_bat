import json
import os
import sys
def findAllFile(base):
    for root, ds, fs in os.walk(base):
        return (fs)
        
if __name__ == '__main__':  
    jsonRoot = sys.argv[1]  
    imgsRoot=sys.argv[2]  
    jsons = findAllFile(jsonRoot) 
    imgs = findAllFile(imgsRoot)
    for img in imgs:
        if img.endswith('.jpg'):
            jsonPath = img+".json"   
            if not jsonPath in jsons:
                imgPath = os.path.join('D:/repo/mvs_mvg_bat/viewer',img) 
                os.remove(imgPath)