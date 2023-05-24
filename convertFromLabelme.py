#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import os
picRoot='E:/viewer'
def scanJson(filePath): 
    files = os.listdir(filePath)
    jsons=[]
    for file in files:
        file_d = os.path.join(filePath, file)
        if os.path.isdir(file_d):  # 如果是文件夹则递归调用 scanDir() 函数
            continue
        if file_d.endswith('.json'):
            jsons.append(file_d)
    return jsons

if __name__ == '__main__':
    jsons= scanJson(picRoot)
    pts={}
    for jsonFile in jsons:
        with open(jsonFile) as f:
            data = json.load(f) 
            imgPath = os.path.join(picRoot,data['imagePath'])
            print(imgPath)
            if not os.path.exists(imgPath):
                print(imgPath,"not exist")
                continue
            for pt in data['shapes']:
                pointName = 'pt'+pt['label']
                pointPos = pt['points'][0] 
                if pointName not in pts : pts[pointName]=[]
                pts[pointName].append({data['imagePath']:pointPos})
    print(pts)
    
    with open("additionalMatch.txt","w") as f:
        for pt in pts:
            f.write(pt+' ') 
            for img in pts[pt]:
                keys = list(img.keys())
                if len(keys)!=1:
                    print("len(keys)!=1")
                    continue 
                f.write(keys[0] + ' '+str('%.4f'%img[keys[0]][0])+ ' '+str('%.4f'%img[keys[0]][1])+' x ') 
            f.write('\n')