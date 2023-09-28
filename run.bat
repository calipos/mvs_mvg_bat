::echo off
set piciturePath=D:\repo\mvs_mvg_bat\viewer
set workSpace=D:\repo\mvs_mvg_bat\viewerout
set workSfmSpace=D:\repo\mvs_mvg_bat\viewerout\sfm
set workMvsSpace=D:\repo\mvs_mvg_bat\viewerout\mvs
set workColmapSpace=D:\repo\mvs_mvg_bat\viewerout\colmap
set workLmsSpace=D:\repo\mvs_mvg_bat\viewerout\landmarks
set workNerfSpace=D:\repo\mvs_mvg_bat\viewerout\nerf
set openMvgPath=D:\repo\openMVG\src\build-2019\installRelease\bin
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%

rd /s /q  %workSpace%

mkdir %workSpace%
mkdir %workSfmSpace% 
mkdir %workLmsSpace% 
mkdir %workMvsSpace% 
mkdir %workColmapSpace% 
mkdir %workNerfSpace% 

echo "-1. prepare images"
::python.exe .\mp42jpg.py  %piciturePath%\8.mp4  2
python.exe .\mp42jpgAndFigureLandmarks.py  %piciturePath%\11.mp4  2  %workLmsSpace%

:: parameter support: ALL FACEOUTLINE LEFTEYEBROW RIGHTEYEBROW NOSEBRIDGE NOSTRIL LEFTEYE RIGHTEYE MOUTH
::bin\DlibLandmark.exe  %piciturePath% %workLmsSpace% FACEOUTLINE LEFTEYEBROW RIGHTEYEBROW NOSEBRIDGE LEFTEYE RIGHTEYE MOUTH 

::python  mediapip.py %piciturePath% %workLmsSpace%

::python deleteImgs.py %workLmsSpace% %piciturePath%

echo "0. Intrinsics analysis"
%openMvgPath%\openMVG_main_SfMInit_ImageListing -i %piciturePath% -o %workSfmSpace%\matches -d  sensor_width_camera_database.txt -f 2000   -c 2

echo "1. Compute features"
::%openMvgPath%\openMVG_main_ComputeFeatures -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches -m AKAZE_MLDB   -p NORMAL
bin\GeneratePipeMediaFeature.exe  %workLmsSpace%  %workSfmSpace%\matches\sfm_data.json %workSfmSpace%\matches

::echo "1.5  replace facial features"
::bin\ReplaceOpenMvgFeature.exe  %workLmsSpace%  %workSfmSpace%\matches\sfm_data.json

::echo "2. Compute pairs"
::%openMvgPath%\openMVG_main_PairGenerator -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches\pairs.txt 

::echo "3. Compute matches"
::%openMvgPath%\openMVG_main_ComputeMatches -i %workSfmSpace%\matches\sfm_data.json -p %workSfmSpace%\matches\pairs.txt -o %workSfmSpace%\matches\matches.putative.bin -n ANNL2  

echo "4. Filter matches"
::%openMvgPath%\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.f.bin  -g f
::%openMvgPath%\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.e.bin  -g e



::echo "5. Incremental reconstruction(sequential)"
::D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Release\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s INCREMENTAL   



echo "6. Incremental reconstruction(GLOBAL)  (INCREMENTAL)"
%openMvgPath%\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s INCREMENTAL        -M  %workSfmSpace%\matches\matches.f.bin  -f  NONE
::            openMVG_main_SfM -i D:\repo\mvs_mvg_bat\viewerout\sfm\matches\sfm_data.json -m D:\repo\mvs_mvg_bat\viewerout\sfm\matches -o D:\repo\mvs_mvg_bat\viewerout\sfm -s INCREMENTAL        -M  D:\repo\mvs_mvg_bat\viewerout\sfm\matches\matches.f.bin  -f  NONE

::echo "7. Export to openMVS"
::echo %openMvgPath%\openMVG_main_openMVG2openMVS -i %workSfmSpace%\sfm_data.bin -o %workMvsSpace%\scene.mvs -d %workMvsSpace%\images   
""%openMvgPath%\openMVG_main_openMVG2openMVS -i %workSfmSpace%\sfm_data.bin -o %workMvsSpace%\scene.mvs -d %workMvsSpace%\images   

echo "7. Export to colmap"
echo %openMvgPath%\openMVG_main_openMVG2Colmap -i %workSfmSpace%\sfm_data.bin -o %workColmapSpace%
%openMvgPath%\openMVG_main_openMVG2Colmap -i %workSfmSpace%\sfm_data.bin -o %workColmapSpace%


bin\ParseMVG.exe   %workSfmSpace%\sfm_data.bin
copy %piciturePath%\*.jpg %workColmapSpace%

exit
 
::run2.bat
::pause

::::openMVG_main_SfM::
::::main_SfM.cpp   if (sfm_engine->Process())
::::                【sfm_global_engine_relative_motions.cpp 112】
::::                KeepOnlyReferencedElement(set_remainingIds, matches_provider_->pairWise_matches_);//可能会修改matches_provider_->pairWise_matches_
::::                Compute_Relative_Rotations(relatives_R);
::::                Compute_Global_Rotations(relatives_R, global_rotations)  //组合相对Rt？
::::                    【sfm_global_engine_relative_motions.cpp 178】
::::                    rotation_averaging_solver.Run
::::                        【GlobalSfM_rotation_averaging.cpp 30】
::::                        list triplet & verify  --> RelativeRotations 个数可能会被排除若干
::::                        重新组合graph 并且KeepLargestBiEdge
::::                        这里重新整理了idx  from ranging in [min(Id), max(Id)] to  [0, nbCam]   
::::                        rotation_averaging::l2::L2RotationAveraging
::::                            【rotation_averaging_l2.cpp 93】 求的线解
::::                        rotation_averaging::l2::L2RotationAveraging_Refine
::::                            【rotation_averaging_l2.cpp 236】 ceres::Solver
::::                        重新整理回原来的idx
::::                    评价角度误差
::::                    输出报告
::::                Compute_Global_Translations .....