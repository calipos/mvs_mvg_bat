import os
import torch
import torch.nn as nn
import torch.nn.functional as F
import data
import nerf
import embedding
import render
import configargparse
import numpy as np
#dataFile='D:/repo/mvs_mvg_bat/viewerout/colmap/img_00017.jpg.rays.bin'
#fx,fy,cx,cy,height,width,worldToCamera,cameraT,xyzs,dirs,rgbs,dists,sigmas = data.readData(dataFile) 

# print(worldToCamera)
# print(cameraT)
# print(xyzs)
# print(dirs)
# print(rgbs)
# print(dists)
# print(sigmas) 
dataDir='D:/repo/mvs_mvg_bat/viewerout/colmap'
n_gpus = torch.cuda.device_count()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DEBUG = False

def get_ray_batch_from_one_image(trainTotalData, args): 
    img_i = np.random.randint(0, len(trainTotalData))
    rayCnt = len(trainTotalData[img_i].dirs) 
    select_coords = np.random.randint(0, rayCnt,[args.N_rand]) 
    rays_o = np.tile(trainTotalData[img_i].cameraT ,[args.N_rand,1])
    rays_o = torch.Tensor(rays_o).to(device)            # (N_rand, 3)
    rays_d = trainTotalData[img_i].dirs[select_coords]  
    rays_d = torch.Tensor(rays_d).to(device)            # (N_rand, 3)
    rays_c = trainTotalData[img_i].rgbs[select_coords]  
    rays_c = torch.Tensor(rays_c).to(device)            # (N_rand, 3)
    return rays_o, rays_d, rays_c,img_i
def batchify(fn, chunk):
    """Constructs a version of 'fn' that applies to smaller batches.
    """
    if chunk is None:
        return fn
    def ret(inputs):
        return torch.cat([fn(inputs[i:i+chunk]) for i in range(0, inputs.shape[0], chunk)], 0)
    return ret    
def run_network(inputs, viewdirs, embedded_cam, fn, embed_fn, embeddirs_fn, bb_center, bb_scale, netchunk=1024*64):
    """Prepares inputs and applies network 'fn'.
    """
    inputs_flat = torch.reshape(inputs, [-1, inputs.shape[-1]])
    inputs_flat = (inputs_flat - bb_center) * bb_scale
    embedded = embed_fn(inputs_flat) # samples * rays, multires * 2 * 3 + 3

    if viewdirs is not None:
        input_dirs = viewdirs[:,None].expand(inputs.shape)
        input_dirs_flat = torch.reshape(input_dirs, [-1, input_dirs.shape[-1]])
        embedded_dirs = embeddirs_fn(input_dirs_flat)
        embedded = torch.cat([embedded, embedded_dirs, embedded_cam.unsqueeze(0).expand(embedded_dirs.shape[0], embedded_cam.shape[0])], -1)

    outputs_flat = batchify(fn, netchunk)(embedded)
    outputs = torch.reshape(outputs_flat, list(inputs.shape[:-1]) + [outputs_flat.shape[-1]])
    return outputs    
def load_checkpoint(args):
    return None
    path = os.path.join(args.ckpt_dir, args.expname)
    ckpts = [os.path.join(path, f) for f in sorted(os.listdir(path)) if '000.tar' in f]
    print('Found ckpts', ckpts)
    ckpt = None
    if len(ckpts) > 0 and not args.no_reload:
        ckpt_path = ckpts[-1]
        print('Reloading from', ckpt_path)
        ckpt = torch.load(ckpt_path)
    return ckpt
def create_nerf(args):
    """Instantiate NeRF's MLP model.
    """
    embed_fn, input_ch = embedding.get_embedder(args.multires, args.i_embed)

    input_ch_views = 0
    embeddirs_fn = None
    if args.use_viewdirs:
        embeddirs_fn, input_ch_views = embedding.get_embedder(args.multires_views, args.i_embed)
    output_ch = 5 if args.N_importance > 0 else 4
    skips = [4]

    model = nerf.NeRF(D=args.netdepth, W=args.netwidth,
                 input_ch=input_ch, output_ch=output_ch, skips=skips,
                 input_ch_views=input_ch_views, input_ch_cam=args.input_ch_cam, use_viewdirs=args.use_viewdirs)
    model = nn.DataParallel(model).to(device)
    grad_vars = list(model.parameters())

    model_fine = None
    if args.N_importance > 0:
        model_fine = nerf.NeRF(D=args.netdepth_fine, W=args.netwidth_fine,
                          input_ch=input_ch, output_ch=output_ch, skips=skips,
                          input_ch_views=input_ch_views, input_ch_cam=args.input_ch_cam, use_viewdirs=args.use_viewdirs)
        model_fine = nn.DataParallel(model_fine).to(device)
        grad_vars += list(model_fine.parameters())

    network_query_fn = lambda inputs, viewdirs, embedded_cam, network_fn : run_network(inputs, viewdirs, embedded_cam, network_fn,
                                                                embed_fn=embed_fn,
                                                                embeddirs_fn=embeddirs_fn,
                                                                bb_center=args.bb_center,
                                                                bb_scale=args.bb_scale,
                                                                netchunk=args.netchunk_per_gpu*args.n_gpus)

    # Create optimizer
    optimizer = torch.optim.Adam(params=grad_vars, lr=args.lrate, betas=(0.9, 0.999))

    start = 0

    ##########################

    # Load checkpoints
    ckpt = load_checkpoint(args)
    if ckpt is not None:
        start = ckpt['global_step']
        optimizer.load_state_dict(ckpt['optimizer_state_dict'])

        # Load model
        model.load_state_dict(ckpt['network_fn_state_dict'])
        if model_fine is not None:
            model_fine.load_state_dict(ckpt['network_fine_state_dict'])

    ##########################
    embedded_cam = torch.tensor((), device=device)
    render_kwargs_train = {
        'network_query_fn' : network_query_fn,
        'embedded_cam' : embedded_cam,
        'perturb' : args.perturb,
        'N_importance' : args.N_importance,
        'network_fine' : model_fine,
        'N_samples' : args.N_samples,
        'network_fn' : model,
        'use_viewdirs' : args.use_viewdirs,
        'raw_noise_std' : args.raw_noise_std,
    }
    render_kwargs_train.update()
    render_kwargs_train['ndc'] = False
    render_kwargs_train['lindisp'] = args.lindisp
    render_kwargs_test = {k : render_kwargs_train[k] for k in render_kwargs_train}
    render_kwargs_test['perturb'] = False
    render_kwargs_test['raw_noise_std'] = 0.
    return render_kwargs_train, render_kwargs_test, start, grad_vars, optimizer

def train_nerf(args):
    np.random.seed(0)
    torch.manual_seed(0)
    if torch.cuda.is_available():torch.cuda.manual_seed(0)
    trainFiles,testFiles,valFiles = data.generateDataJson(dataDir)
    if len(trainFiles) == 0:        print("Error: There is no trainFiles")
    else:print("train file cnt",len(trainFiles))
    if len(testFiles) == 0:        print("Warning: There is no testFiles")
    else:print("test file cnt",len(testFiles))
    if len(valFiles) == 0:        print("Warning: There is no valFiles")
    else:print("val file cnt",len(valFiles))
    near, far = 0,1  ###!
    
    # create nerf model
    render_kwargs_train, render_kwargs_test, start, nerf_grad_vars, optimizer = create_nerf(args)

    # keep test data on cpu until needed 
    fx,fy,cx,cy,height,width,trainTotalData = data.readTotalData(dataDir,trainFiles)
    # _,_,_,_,_,_,testTotalData = data.readTotalData(dataDir,testFiles)
    # _,_,_,_,_,_,valTotalData = data.readTotalData(dataDir,valFiles)
    print(fx,fy,cx,cy,height,width)
 
    # create camera embedding function
    embedcam_fn = None
    if args.input_ch_cam > 0:        embedcam_fn = torch.nn.Embedding(len(trainFiles), args.input_ch_cam)

    # optimize nerf
    print('Begin')
    N_iters = 500000 + 1
    global_step = start
    start = start + 1
    for i in range(start, N_iters):
        # update learning rate
        if i > args.start_decay_lrate and i <= args.end_decay_lrate:
            portion = (i - args.start_decay_lrate) / (args.end_decay_lrate - args.start_decay_lrate)
            decay_rate = 0.1
            new_lrate = args.lrate * (decay_rate ** portion)
            nerf.update_learning_rate(optimizer, new_lrate)
        
        # make batch        
        rays_o, rays_d, rays_c,img_i=get_ray_batch_from_one_image(trainTotalData, args)
        if args.input_ch_cam > 0:            render_kwargs_train['embedded_cam'] = embedcam_fn(torch.tensor(img_i, device=device))


        # render
        rgb, _, _, extras = render.render(height,width, None, chunk=args.chunk, rays=rays_o, verbose=i < 10, retraw=True, **render_kwargs_train)
        

        global_step += 1

def config_parser():
    parser = configargparse.ArgumentParser()
    parser.add_argument('--task', type=str, default="train",                        help='one out of: "train", "test", "test_with_opt", "video"')
    parser.add_argument('--config', is_config_file=True, 
                        help='config file path')
    parser.add_argument("--expname", type=str, default=None, 
                        help='specify the experiment, required for "test" and "video", optional for "train"')

    # training options
    parser.add_argument("--netdepth", type=int, default=8, 
                        help='layers in network')
    parser.add_argument("--netwidth", type=int, default=256, 
                        help='channels per layer')
    parser.add_argument("--netdepth_fine", type=int, default=8, 
                        help='layers in fine network')
    parser.add_argument("--netwidth_fine", type=int, default=256, 
                        help='channels per layer in fine network')
    parser.add_argument("--N_rand", type=int, default=32*32,
                        help='batch size (number of random rays per gradient step)')
    parser.add_argument("--lrate", type=float, default=5e-4, 
                        help='learning rate')
    parser.add_argument("--start_decay_lrate", type=int, default=400000, 
                        help='start iteration for learning rate decay')
    parser.add_argument("--end_decay_lrate", type=int, default=500000, 
                        help='end iteration for learning rate decay')
    parser.add_argument("--chunk", type=int, default=1024*32, 
                        help='number of rays processed in parallel, decrease if running out of memory')
    parser.add_argument("--netchunk_per_gpu", type=int, default=1024*64*4, 
                        help='number of pts sent through network in parallel, decrease if running out of memory')
    parser.add_argument("--no_reload", action='store_true', 
                        help='do not reload weights from saved ckpt')
    parser.add_argument("--depth_loss_weight", type=float, default=0.004,
                        help='weight of the depth loss, values <=0 do not apply depth loss')
    parser.add_argument("--invalidate_large_std_threshold", type=float, default=1.,
                        help='invalidate completed depth values with standard deviation greater than threshold in m, \
                            thresholds <=0 deactivate invalidation')
    parser.add_argument("--depth_completion_input_h", type=int, default=240, 
                        help='depth completion network input height')
    parser.add_argument("--depth_completion_input_w", type=int, default=320, 
                        help='depth completion network input width')

    # rendering options
    parser.add_argument("--N_samples", type=int, default=256,
                        help='number of coarse samples per ray')
    parser.add_argument("--N_importance", type=int, default=0,
                        help='number of additional fine samples per ray')
    parser.add_argument("--perturb", type=float, default=1.,
                        help='set to 0. for no jitter, 1. for jitter')
    parser.add_argument("--use_viewdirs", action='store_true', default=True,
                        help='use full 5D input instead of 3D')
    parser.add_argument("--i_embed", type=int, default=0, 
                        help='set 0 for default positional encoding, -1 for none')
    parser.add_argument("--multires", type=int, default=9,
                        help='log2 of max freq for positional encoding (3D location)')
    parser.add_argument("--multires_views", type=int, default=0,
                        help='log2 of max freq for positional encoding (2D direction)')
    parser.add_argument("--input_ch_cam", type=int, default=4,
                        help='number of channels for camera index embedding')
    parser.add_argument("--raw_noise_std", type=float, default=0., 
                        help='std dev of noise added to regularize sigma_a output, 1e0 recommended')
    parser.add_argument("--lindisp", action='store_true', default=False,
                        help='sampling linearly in disparity rather than depth')

    # logging/saving options
    parser.add_argument("--i_print",   type=int, default=1000, 
                        help='frequency of console printout and metric logging')
    parser.add_argument("--i_img",     type=int, default=20000,
                        help='frequency of tensorboard image logging')
    parser.add_argument("--i_weights", type=int, default=100000,
                        help='frequency of weight ckpt saving')
    parser.add_argument("--ckpt_dir", type=str, default="",
                        help='checkpoint directory')

    # data options
    parser.add_argument("--scene_id", type=str, default="scene0710_00",
                        help='scene identifier')
    parser.add_argument("--depth_prior_network_path", type=str, default="",
                        help='path to the depth prior network checkpoint to be used')
    parser.add_argument("--data_dir", type=str, default="",
                        help='directory containing the scenes')

    return parser

def run_nerf():
    parser = config_parser()
    args = parser.parse_args()
    print(f"Using {n_gpus} GPU(s).")

    
    # Compute boundaries of 3D space
    max_xyz = torch.full((3,), -1e6)
    min_xyz = torch.full((3,), 1e6)
    if args.task == "train":
        train_nerf(args)
        exit()
    # create nerf model for testing
    #_, render_kwargs_test, _, nerf_grad_vars, _ = create_nerf(args, scene_sample_params)

if __name__=='__main__':
    if torch.cuda.is_available():torch.set_default_tensor_type('torch.cuda.FloatTensor')

    run_nerf()
