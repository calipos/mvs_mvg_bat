import sys
sys.path.append('core')

import argparse
import os
import cv2
import glob
import numpy as np
import torch
from PIL import Image

from raft import RAFT
from utils import flow_viz
from utils.utils import InputPadder
from tensorboardX import SummaryWriter  # 用于进行可视化
from torchviz import make_dot

DEVICE = 'cuda'   #'cuda'  cpu

def load_image(imfile):
    img = np.array(Image.open(imfile)).astype(np.uint8)
    img = torch.from_numpy(img).permute(2, 0, 1).float()
    return img[None].to(DEVICE)

saveIdx=0
def viz(img, flo):
    img = img[0].permute(1,2,0).cpu().numpy()
    flo = flo[0].permute(1,2,0).cpu().numpy()
    
    # map flow to rgb image
    flo = flow_viz.flow_to_image(flo)
    img_flo = np.concatenate([img, flo], axis=1)

    global saveIdx
    cv2.imwrite("%d.jpg"%(saveIdx),img_flo[:, :, [2,1,0]])
    saveIdx+=1
    # cv2.namedWindow('image',0)
    # cv2.imshow('image', img_flo[:, :, [2,1,0]]/255.0)
    # cv2.waitKey()


def demo(args):
    model = torch.nn.DataParallel(RAFT(args))
    model.load_state_dict(torch.load(args.model))

    model = model.module
    model.to(DEVICE)
    model.eval()

    # sampledata1 = torch.rand(1, 3, 400, 600)
    # sampledata2 = torch.rand(1, 3, 400, 600) 
    # sampledata1 =sampledata1.to(DEVICE)
    # sampledata2 =sampledata2.to(DEVICE)
    # out = model(sampledata1,sampledata2,iters=3, test_mode=True)  
    # g = make_dot(out)
    # g.render('modelviz', view=False )  # 这种方式会生成一个pdf文件


    with torch.no_grad():
        images = glob.glob(os.path.join(args.path, '*.png')) + \
                 glob.glob(os.path.join(args.path, '*.jpg'))
        
        images = sorted(images)
        for imfile1, imfile2 in zip(images[:-1], images[1:]):
            image1 = load_image(imfile1)
            image2 = load_image(imfile2)

            padder = InputPadder(image1.shape)
            image1, image2 = padder.pad(image1, image2)

            flow_low, flow_up = model(image1, image2, iters=20, test_mode=True)
            file = open("%d.txt"%(saveIdx), "wb") 
            file.write(flow_up[0].permute(1,2,0).cpu().numpy() .reshape(-1)) 
            file.close()
            viz(image1, flow_up)      
            


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help="restore checkpoint",default='./model/raft-things.pth')
    parser.add_argument('--path', help="dataset for evaluation",default='../viewer')
    parser.add_argument('--small', action='store_true', help='use small model')
    parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
    parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
    args = parser.parse_args()

    demo(args)
