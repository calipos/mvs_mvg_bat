import xml.etree.ElementTree as ET
from gener_new_pair import getImgPathsFromJson 
jsonPath='E:/viewerout/sfm/matches/sfm_data.json' 
svgPath='E:/viewerout/sfm/global_relative_rotation_pose_graph_final.svg' 
fileList,fileIdx,fileIdx2 = getImgPathsFromJson(jsonPath)
print(fileIdx2)

tree = ET.parse(svgPath)
root = tree.getroot()
for child in root:
     print(child.attrib)  
     for child2 in child: 
        if('id' in child2.attrib and child2.attrib['class'] == 'edge'): 
            for child3 in child2:          
                link = child3.text  
                link = link.replace("-", " ").replace("n", " ")
                link = ' '.join(link.split())
                pair = link.split(' ',-1)  
                print(fileIdx2[int(pair[0])],' -- ',fileIdx2[int(pair[1])])
                break

 