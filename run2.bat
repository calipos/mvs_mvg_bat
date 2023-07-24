::secho off
set piciturePath=D:\repo\mvs_mvg_bat\viewer
set workSpace=D:\repo\mvs_mvg_bat\viewerout
set workSfmSpace=D:\repo\mvs_mvg_bat\viewerout\sfm
set workMvsSpace=D:\repo\mvs_mvg_bat\viewerout\mvs
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%
set openMvsPath=D:\repo\openMVS\build-vs2019\install\bin\OpenMVS

xcopy /y .\Densify.ini %workMvsSpace%

echo "12. Densify point cloud"
REM Generic options:
  REM -h [ --help ]                         produce this help message
  REM -w [ --working-folder ] arg           working directory (default current
                                        REM directory)
  REM -c [ --config-file ] arg (=DensifyPointCloud.cfg)
                                        REM file name containing program options
  REM --archive-type arg (=3)               project archive type: 0-text, 1-binary,
                                        REM 2-compressed binary
  REM --process-priority arg (=-1)          process priority (below normal by
                                        REM default)
  REM --max-threads arg (=1)                maximum number of threads (0 for using
                                        REM all available cores)
  REM -v [ --verbosity ] arg (=2)           verbosity level

REM Densify options:
  REM -i [ --input-file ] arg               input filename containing camera poses
                                        REM and image list
  REM -o [ --output-file ] arg              output filename for storing the dense
                                        REM point-cloud (optional)
  REM --view-neighbors-file arg             input filename containing the list of
                                        REM views and their neighbors (optional)
  REM --output-view-neighbors-file arg      output filename containing the
                                        REM generated list of views and their
                                        REM neighbors
  REM --resolution-level arg (=1)           how many times to scale down the images
                                        REM before point cloud computation
  REM --max-resolution arg (=2560)          do not scale images higher than this
                                        REM resolution
  REM --min-resolution arg (=640)           do not scale images lower than this
                                        REM resolution
  REM --sub-resolution-levels arg (=2)      number of patch-match sub-resolution
                                        REM iterations (0 - disabled)
  REM --number-views arg (=5)               number of views used for depth-map
                                        REM estimation (0 - all neighbor views
                                        REM available)
  REM --number-views-fuse arg (=3)          minimum number of images that agrees
                                        REM with an estimate during fusion in order
                                        REM to consider it inlier (<2 - only merge
                                        REM depth-maps)
  REM --ignore-mask-label arg (=-1)         integer value for the label to ignore
                                        REM in the segmentation mask (<0 -
                                        REM disabled)
  REM --iters arg (=3)                      number of patch-match iterations
  REM --geometric-iters arg (=2)            number of geometric consistent
                                        REM patch-match iterations (0 - disabled)
  REM --estimate-colors arg (=2)            estimate the colors for the dense
                                        REM point-cloud (0 - disabled, 1 - final, 2
                                        REM - estimate)
  REM --estimate-normals arg (=2)           estimate the normals for the dense
                                        REM point-cloud (0 - disabled, 1 - final, 2
                                        REM - estimate)
  REM --sub-scene-area arg (=0)             split the scene in sub-scenes such that
                                        REM each sub-scene surface does not exceed
                                        REM the given maximum sampling area (0 -
                                        REM disabled)
  REM --sample-mesh arg (=0)                uniformly samples points on a mesh (0 -
                                        REM disabled, <0 - number of points, >0 -
                                        REM sample density per square unit)
  REM --fusion-mode arg (=0)                depth-maps fusion mode (-2 - fuse
                                        REM disparity-maps, -1 - export
                                        REM disparity-maps only, 0 - depth-maps &
                                        REM fusion, 1 - export depth-maps only)
  REM --postprocess-dmaps arg (=7)          flags used to filter the depth-maps
                                        REM after estimation (0 - disabled, 1 -
                                        REM remove-speckles, 2 - fill-gaps, 4 -
                                        REM adjust-filter)
  REM --filter-point-cloud arg (=0)         filter dense point-cloud based on
                                        REM visibility (0 - disabled)
  REM --export-number-views arg (=0)        export points with >= number of views
                                        REM (0 - disabled, <0 - save MVS project
                                        REM too)
  REM --roi-border arg (=0)                 add a border to the region-of-interest
                                        REM when cropping the scene (0 - disabled,
                                        REM >0 - percentage, <0 - absolute)
  REM --estimate-roi arg (=2)               estimate and set region-of-interest (0
                                        REM - disabled, 1 - enabled, 2 - adaptive)
  REM --crop-to-roi arg (=1)                crop scene using the region-of-interest
  REM --remove-dmaps arg (=0)               remove depth-maps after fusion
%openMvsPath%\DensifyPointCloud.exe %workMvsSpace%/scene.mvs --dense-config-file %workMvsSpace%/Densify.ini --resolution-level 1 --number-views 8 -w "%workMvsSpace%"  --verbosity 6    --min-resolution 960  --estimate-normals 0   --iters 0
::
echo "13. Reconstruct the mesh"
%openMvsPath%\ReconstructMesh scene_dense.mvs -w "%workMvsSpace%"

::echo "14. Refine the mesh"
::RefineMesh %workMvsSpace%/scene_dense_mesh.mvs --scales 1 --gradient-step 25.05 -w "%workMvsSpace%"

::echo "15. Texture the mesh"
::TextureMesh %workMvsSpace%/scene_dense_mesh.mvs --decimate 0.5 -w "%workMvsSpace%"

::pause




  