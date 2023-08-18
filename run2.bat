::secho off
set piciturePath=D:\repo\mvs_mvg_bat\viewer
set workSpace=D:\repo\mvs_mvg_bat\viewerout
set workSfmSpace=D:\repo\mvs_mvg_bat\viewerout\sfm
set workMvsSpace=D:\repo\mvs_mvg_bat\viewerout\mvs
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%
cd RAFT

python demo.py  --model=./model/raft-things.pth    --path=../viewer
::pause




  