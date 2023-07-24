import time
import numpy as np
import io
import os
from PIL import Image
import cv2
import saverloader
import imageio.v2 as imageio
from nets.pips import Pips
import utils.improc
import random
import glob
import torch
import torch.nn.functional as F
from tensorboardX import SummaryWriter

def run_model2(model, rgbs, xy0):
    rgbs = rgbs.float() # B, S, C, H, W

    B, S, C, H, W = rgbs.shape
    rgbs_ = rgbs.reshape(B*S, C, H, W)
    H_, W_ = H, W  # 360, 640
    rgbs_ = F.interpolate(rgbs_, (H_, W_), mode='bilinear')
    H, W = H_, W_
    rgbs = rgbs_.reshape(B, S, C, H, W)
    
    B1,N,dim1 = xy0.shape
    assert(dim1==2)
    assert(B1==B)
    _, S, C, H, W = rgbs.shape

    cur_frame = 0
    done = False
    traj_e = torch.zeros((B, S, N,2), dtype=torch.float32, device='cpu')
    traj_e[:,0] = xy0 # B,N,2 
    feat_init = None
    while not done:
        end_frame = cur_frame + 8
        if end_frame>S:end_frame=S
        rgb_seq = rgbs[:,cur_frame:end_frame]
        S_local = rgb_seq.shape[1]
        rgb_seq = torch.cat([rgb_seq, rgb_seq[:,-1].unsqueeze(1).repeat(1,8-S_local,1,1,1)], dim=1)
    

        outs = model(traj_e[:,cur_frame], rgb_seq, iters=6, feat_init=None, return_feat=True)
        preds = outs[0]
        vis = outs[2] # B, S, 1
        feat_init = outs[3]

        vis = torch.sigmoid(vis) # visibility confidence
        xys = preds[-1].reshape(1, 8,N, 2)
        traj_e[:,cur_frame:end_frame] = xys[:,:S_local]

        found_skip = False
        thr = 0.9
        si_last = 8-1 # last frame we are willing to take
        si_earliest = 1 # earliest frame we are willing to take
        si = si_last
        # while not found_skip:
        #     if vis[0,si] > thr:
        #         found_skip = True
        #     else:
        #         si -= 1
        #     if si == si_earliest:
        #         thr -= 0.02
        #         si = si_last
        cur_frame = cur_frame + 8
        if cur_frame >= S:
            done = True
    pad = 50
    rgbs = F.pad(rgbs.reshape(B*S, 3, H, W), (pad, pad, pad, pad), 'constant', 0).reshape(B, S, 3, H+pad*2, W+pad*2)
    trajs_e = trajs_e + pad
    prep_rgbs = utils.improc.preprocess_color(rgbs)
    gray_rgbs = torch.mean(prep_rgbs, dim=2, keepdim=True).repeat(1, 1, 3, 1, 1)
    
    writer_t = SummaryWriter('./00/t', max_queue=10, flush_secs=60)
    sw = utils.improc.Summ_writer(
        writer=writer_t,
        global_step=0,
        log_freq=10,
        fps=12,
        scalar_freq=int(log_freq/2),
        just_gif=True)

    kp_vis = sw.summ_traj2ds_on_rgbs('video_%d/kp_%d_trajs_e_on_rgbs' % (sw.global_step, n), trajs_e[0:1,:,n:n+1], gray_rgbs[0:1,:S], cmap='spring', linewidth=linewidth)
    # write to disk, in case that's more convenient
    kp_list = list(kp_vis.unbind(1))
    kp_list = [kp[0].permute(1,2,0).cpu().numpy() for kp in kp_list]
    kp_list = [Image.fromarray(kp) for kp in kp_list]
    out_fn = './chain_out_%d.gif' % sw.global_step
    kp_list[0].save(out_fn, save_all=True, append_images=kp_list[1:])
    print('saved %s' % out_fn)
    
    return  trajs_e-pad


def run_model(model, rgbs, N):
    rgbs = rgbs.float() # B, S, C, H, W

    B, S, C, H, W = rgbs.shape
    rgbs_ = rgbs.reshape(B*S, C, H, W)
    H_, W_ = 360, 640
    rgbs_ = F.interpolate(rgbs_, (H_, W_), mode='bilinear')
    H, W = H_, W_
    rgbs = rgbs_.reshape(B, S, C, H, W)

    # try to pick a point on the dog, so we get an interesting trajectory 
    x = torch.ones((1, N), device=torch.device('cpu')) * 450.0
    y = torch.ones((1, N), device=torch.device('cpu')) * 150.0
    xy0 = torch.stack([x, y], dim=-1) # B, N, 2
    _, S, C, H, W = rgbs.shape

    trajs_e = torch.zeros((B, S, N, 2), dtype=torch.float32, device='cpu')
    for n in range(N):
        # print('working on keypoint %d/%d' % (n+1, N))
        cur_frame = 0
        done = False
        traj_e = torch.zeros((B, S, 2), dtype=torch.float32, device='cpu')
        traj_e[:,0] = xy0[:,n] # B, 1, 2  # set first position 
        feat_init = None
        while not done:
            end_frame = cur_frame + 8

            rgb_seq = rgbs[:,cur_frame:end_frame]
            S_local = rgb_seq.shape[1]
            rgb_seq = torch.cat([rgb_seq, rgb_seq[:,-1].unsqueeze(1).repeat(1,8-S_local,1,1,1)], dim=1)

            outs = model(traj_e[:,cur_frame].reshape(1, -1, 2), rgb_seq, iters=6, feat_init=feat_init, return_feat=True)
            preds = outs[0]
            vis = outs[2] # B, S, 1
            feat_init = outs[3]
            
            vis = torch.sigmoid(vis) # visibility confidence
            xys = preds[-1].reshape(1, 8, 2)
            traj_e[:,cur_frame:end_frame] = xys[:,:S_local]

            found_skip = False
            thr = 0.9
            si_last = 8-1 # last frame we are willing to take
            si_earliest = 1 # earliest frame we are willing to take
            si = si_last
            while not found_skip:
                if vis[0,si] > thr:
                    found_skip = True
                else:
                    si -= 1
                if si == si_earliest:
                    # print('decreasing thresh')
                    thr -= 0.02
                    si = si_last
            # print('found skip at frame %d, where we have' % si, vis[0,si].detach().item())

            cur_frame = cur_frame + si

            if cur_frame >= S:
                done = True
        trajs_e[:,:,n] = traj_e
    
    pad = 50
    rgbs = F.pad(rgbs.reshape(B*S, 3, H, W), (pad, pad, pad, pad), 'constant', 0).reshape(B, S, 3, H+pad*2, W+pad*2)
    trajs_e = trajs_e + pad

    prep_rgbs = utils.improc.preprocess_color(rgbs)
    gray_rgbs = torch.mean(prep_rgbs, dim=2, keepdim=True).repeat(1, 1, 3, 1, 1)
    


    return trajs_e-pad
    
if __name__ == '__main__':    
    exp_name = '00' # (exp_name is used for logging notes that correspond to different runs)
    init_dir = './reference_model'

    ## choose hyps
    B = 1
    S = 50
    xy0=torch.tensor([[450,150],[500,200]])
    xy0=xy0.reshape(B,-1,2)
    N = xy0.shape[1]  # number of points to track

    filenames = glob.glob('../viewer/*.jpg')
    filenames = sorted(filenames)
    print('filenames', filenames)
    max_iters = len(filenames)//(S//2)-1 # run slightly overlapping subseqs

    log_freq = 1 # when to produce visualizations 

    ## autogen a name
    model_name = "%02d_%d_%d" % (B, S, N)
    model_name += "_%s" % exp_name
    import datetime
    model_date = datetime.datetime.now().strftime('%H%M%S')
    model_name = model_name + '_' + model_date
    print('model_name', model_name)

    log_dir = 'logs_chain_demo'

    global_step = 0

    model = Pips(stride=2)
    parameters = list(model.parameters())
    if init_dir:
        _ = saverloader.load(init_dir, model)
    global_step = 0
    model.eval()
    print('max_iters=',max_iters)
    while global_step < max_iters:
        
        read_start_time = time.time()
        
        global_step += 1

        try:
            rgbs = []
            for s in range(S):
                fn = filenames[(global_step-1)*S//2+s]
                if s==0:
                    print('start frame', fn)
                im = imageio.imread(fn)
                im = im.astype(np.uint8)
                rgbs.append(torch.from_numpy(im).permute(2,0,1))
            rgbs = torch.stack(rgbs, dim=0).unsqueeze(0) # 1, S, C, H, W

            read_time = time.time()-read_start_time
            iter_start_time = time.time()

            with torch.no_grad():
                trajs_e = run_model2(model, rgbs, xy0)

            iter_time = time.time()-iter_start_time
            print('%s; step %06d/%d; rtime %.2f; itime %.2f' % (
                model_name, global_step, max_iters, read_time, iter_time))
        except FileNotFoundError as e:
            print('error', e)
            
