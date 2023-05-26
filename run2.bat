echo off
set piciturePath=D:\repo\mvs_mvg_bat\viewer
set workSpace=D:\repo\mvs_mvg_bat\viewerout
set workSfmSpace=D:\repo\mvs_mvg_bat\viewerout\sfm
set workMvsSpace=D:\repo\mvs_mvg_bat\viewerout\mvs
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%

mkdir %workMvsSpace% 

echo "11. Export to openMVS"
openMVG_main_openMVG2openMVS -i %workSfmSpace%/sfm_data.bin -o %workMvsSpace%/scene.mvs -d %workMvsSpace%/images   

echo "12. Densify point cloud"
DensifyPointCloud.exe %workMvsSpace%/scene.mvs --dense-config-file %workMvsSpace%/Densify.ini --resolution-level 1 --number-views 8 -w "%workMvsSpace%"
::
echo "13. Reconstruct the mesh"
ReconstructMesh scene_dense.mvs -w "%workMvsSpace%"

::echo "14. Refine the mesh"
::RefineMesh %workMvsSpace%/scene_dense_mesh.mvs --scales 1 --gradient-step 25.05 -w "%workMvsSpace%"

::echo "15. Texture the mesh"
::TextureMesh %workMvsSpace%/scene_dense_mesh.mvs --decimate 0.5 -w "%workMvsSpace%"

::pause




  