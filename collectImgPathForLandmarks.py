import json
import os
import sys

def readImagePathFromSfmJson(path):
    paths=[]
    with open(path) as f:
        data = json.load(f)
        imgRoot = data['root_path'] 
        views = data['views']
        for view in views:
            imgPath = os.path.join(imgRoot,view['value']['ptr_wrapper']['data']['filename'])
            paths.append(imgPath)
    return paths
if __name__ == '__main__':  
    sfmJsonPath = sys.argv[1]  
    imagePathSetPath=sys.argv[2] 
    jsonRoot=sys.argv[3] 
    paths = readImagePathFromSfmJson(sfmJsonPath)  
    with open(imagePathSetPath,'w') as f:
        for p in paths:
            fileName = os.path.basename(p) 
            f.write(p+'\n')
            f.write(os.path.join(jsonRoot,fileName+'.json')+'\n')            
 