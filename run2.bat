::echo off
set piciturePath=D:\repo\mvs_mvg_bat\viewer
set workSpace=D:\repo\mvs_mvg_bat\viewerout
set workSfmSpace=D:\repo\mvs_mvg_bat\viewerout\sfm
set workMvsSpace=D:\repo\mvs_mvg_bat\viewerout\mvs
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%
set openMvsPath=D:\repo\openMVS\build-vs2019\install\bin\OpenMVS


echo "12. Densify point cloud"
%openMvsPath%\DensifyPointCloud.exe %workMvsSpace%/scene.mvs --dense-config-file %workMvsSpace%/Densify.ini --resolution-level 1 --number-views 8 -w "%workMvsSpace%"
::
echo "13. Reconstruct the mesh"
%openMvsPath%\ReconstructMesh scene_dense.mvs -w "%workMvsSpace%"

::echo "14. Refine the mesh"
::RefineMesh %workMvsSpace%/scene_dense_mesh.mvs --scales 1 --gradient-step 25.05 -w "%workMvsSpace%"

::echo "15. Texture the mesh"
::TextureMesh %workMvsSpace%/scene_dense_mesh.mvs --decimate 0.5 -w "%workMvsSpace%"

::pause




  