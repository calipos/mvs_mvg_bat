import json
import os
import sys

#print ('参数个数为:', len(sys.argv), '个参数。')
#print ('参数列表:', str(sys.argv))
#print ('脚本名:', str(sys.argv[0]))
#print ('pairsFile:', str(sys.argv[1]))

def getImgPathsFromJson(jsonPath):
    files=[]
    fileIdxFromJson=[]
    fileIdxFromPath=[]
    pathIdxMap={}
    jsonIdxMap={}
    jsonIdxMap2={}
    with open(jsonPath) as f:
        data = json.load(f)  
        views = data['views']
        imgCnt = len(views)
        for img in views:
            imgPath = img['value']['ptr_wrapper']['data']['filename']
            name1 = imgPath.split('.',-1) 
            if len(name1)<2:
                print('err : len(name1)')
                exit(0)
            name2 = name1[-2].split('_',-1) 
            if len(name2)<2:
                print('err : len(name2)') 
            idxFromPath = int(name2[-1])
            idxFromJson = img['key']
            #print(imgPath,idxFromName) 
            fileIdxFromJson.append(idxFromJson)
            fileIdxFromPath.append(idxFromPath)
            pathIdxMap[idxFromPath] = img['value']['ptr_wrapper']['data']['filename']
            jsonIdxMap[img['value']['ptr_wrapper']['data']['filename']] = idxFromJson
            jsonIdxMap2[idxFromJson]=img['value']['ptr_wrapper']['data']['filename']
    fileIdxFromPath.sort() 
    for idx in fileIdxFromPath:
        files.append(pathIdxMap[idx]) 
    return files,jsonIdxMap,jsonIdxMap2
    
def getIdx(files,fileMap):
    idx=[]
    for file in files:
        if file not in fileMap:
            print("err : ",file)
        idx.append(fileMap[file])
    return idx
def getPairs(cluster):
    pairs=[]
    for i in cluster:
        for j in cluster:
            if i>=j:continue
            pairs.append([i,j])
    return pairs

if __name__ == '__main__':

    sfm_json_path=str(sys.argv[1])

    fileList,fileIdx,fileIdx2 = getImgPathsFromJson(sfm_json_path)
    print(fileList)
    print(fileIdx) 
    input("Please press the Enter key to proceed")
    cluster1 = ['im_0.jpg', 'im_1.jpg', 'im_2.jpg', 'im_3.jpg', 'im_4.jpg', 'im_5.jpg', 'im_6.jpg', 'im_7.jpg', 'im_8.jpg', 'im_9.jpg', 'im_10.jpg', 'im_11.jpg', 'im_12.jpg', 'im_13.jpg', 'im_14.jpg', 'im_15.jpg', 'im_16.jpg', 'im_17.jpg', 'im_18.jpg', 'im_19.jpg', 'im_20.jpg', 'im_21.jpg', 'im_22.jpg', 'im_23.jpg', 'im_24.jpg', 'im_25.jpg', 'im_26.jpg', 'im_27.jpg', 'im_28.jpg', 'im_29.jpg', 'im_30.jpg', 'im_31.jpg', 'im_32.jpg', 'im_33.jpg', 'im_34.jpg', 'im_35.jpg', 'im_36.jpg', 'im_37.jpg', 'im_38.jpg', 'im_39.jpg', 'im_40.jpg', 'im_41.jpg', 'im_42.jpg', 'im_43.jpg', 'im_44.jpg', 'im_45.jpg', 'im_46.jpg', 'im_47.jpg', 'im_48.jpg']
    cluster2 = [ 'im_49.jpg', 'im_50.jpg', 'im_51.jpg', 'im_52.jpg', 'im_53.jpg', 'im_54.jpg', 'im_55.jpg', 'im_56.jpg', 'im_57.jpg', 'im_58.jpg', 'im_59.jpg', 'im_60.jpg', 'im_61.jpg', 'im_62.jpg', 'im_63.jpg', 'im_64.jpg', 'im_65.jpg', 'im_66.jpg', 'im_67.jpg', 'im_68.jpg', 'im_69.jpg', 'im_70.jpg', 'im_71.jpg', 'im_72.jpg', 'im_73.jpg', 'im_74.jpg', 'im_75.jpg', 'im_76.jpg', 'im_77.jpg', 'im_78.jpg', 'im_79.jpg', 'im_80.jpg', 'im_81.jpg', 'im_82.jpg', 'im_83.jpg', 'im_84.jpg', 'im_85.jpg', 'im_86.jpg', 'im_87.jpg', 'im_88.jpg', 'im_89.jpg', 'im_90.jpg', 'im_91.jpg', 'im_92.jpg', 'im_93.jpg', 'im_94.jpg', 'im_95.jpg', 'im_96.jpg', 'im_97.jpg', 'im_98.jpg', 'im_99.jpg', 'im_100.jpg', 'im_101.jpg', 'im_102.jpg', 'im_103.jpg', 'im_104.jpg', 'im_105.jpg', 'im_106.jpg', 'im_107.jpg', 'im_108.jpg', 'im_109.jpg', 'im_110.jpg', 'im_111.jpg', 'im_112.jpg', 'im_113.jpg', 'im_114.jpg', 'im_115.jpg', 'im_116.jpg', 'im_117.jpg', 'im_118.jpg', 'im_119.jpg', 'im_120.jpg', 'im_121.jpg', 'im_122.jpg', 'im_123.jpg', 'im_124.jpg', 'im_125.jpg', 'im_126.jpg', 'im_127.jpg', 'im_128.jpg', 'im_129.jpg', 'im_130.jpg', 'im_131.jpg', 'im_132.jpg', 'im_133.jpg', 'im_134.jpg', 'im_135.jpg']
    #cluster3 = ['im_48.jpg', 'im_108.jpg'] 
    #cluster4 = ['im_19.jpg', 'im_128.jpg'] 
    #cluster5 = ['im_48.jpg', 'im_128.jpg'] 
 
    #pairs1 = getPairs(getIdx(cluster1,fileIdx))
    pairs2 = getPairs(getIdx(cluster2,fileIdx)) 
    #pairs3 = getPairs(getIdx(cluster3,fileIdx))   
    #pairs4 = getPairs(getIdx(cluster4,fileIdx))   
    #pairs5 = getPairs(getIdx(cluster5,fileIdx))   
    total_pairs=[]
    #total_pairs.extend(pairs1)
    total_pairs.extend(pairs2)
    #total_pairs.extend(pairs3)  
    #total_pairs.extend(pairs4)  
    #total_pairs.extend(pairs5)   
    with open(str(sys.argv[2]),"w") as f:
        for i in range(len(total_pairs)):
            f.write(str(total_pairs[i][0])+' '+str(total_pairs[i][1])+'\n') 
    with open(str(sys.argv[2])+".txt","w") as f:
        for i in range(len(total_pairs)):
            f.write(fileIdx2[total_pairs[i][0]]+' '+fileIdx2[total_pairs[i][1]]+'\n') 