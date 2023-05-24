::@echo off
set piciturePath=E:\viewer
set workSpace=E:\viewerout
set workSfmSpace=E:\viewerout\sfm
set workMvsSpace=E:\viewerout\mvs
set workLmsSpace=E:\viewerout\landmarks
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%

rd /s /q  %workSpace%

mkdir %workSpace%
mkdir %workSfmSpace% 
mkdir %workLmsSpace% 

echo "0. Intrinsics analysis"
D:\repo\OpenMVG.release\openMVG_main_SfMInit_ImageListing -i %piciturePath% -o %workSfmSpace%\matches -d D:\repo\OpenMVG.release\sensor_width_camera_database.txt -f 2000

python.exe .\collectImgPathForLandmarks.py %workSfmSpace%\matches\sfm_data.json  imagePathSet.txt %workLmsSpace%
.\WorldBuild\x64\Release\DlibLandmark.exe  imagePathSet.txt


echo "1. Compute features"
D:\repo\OpenMVG.release\openMVG_main_ComputeFeatures -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches -m SIFT   -p NORMAL

exit

echo "2. Compute pairs"
D:\repo\OpenMVG.release\openMVG_main_PairGenerator -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches\pairs.txt
::python.exe .\gener_new_pair.py  %workSfmSpace%\matches\sfm_data.json  %workSfmSpace%\matches\pairs.txt


echo "3. Compute matches"
D:\repo\OpenMVG.release\openMVG_main_ComputeMatches -i %workSfmSpace%\matches\sfm_data.json -p %workSfmSpace%\matches\pairs.txt -o %workSfmSpace%\matches\matches.putative.bin -n ANNL2
::exit

echo "4. Filter matches"
D:\repo\OpenMVG.release\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.f.bin  -g f
::D:\repo\OpenMVG.release\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.fe.bin  -g e



::echo "5. Incremental reconstruction(sequential)"
::D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Release\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s INCREMENTAL   



echo "6. Incremental reconstruction(global)"
D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Release\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s GLOBAL        -M  %workSfmSpace%\matches\matches.f.bin

run2.bat
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