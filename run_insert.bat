::@echo off
set piciturePath=E:\viewer
set workSpace=E:\viewerout
set workSfmSpace=E:\viewerout\sfm
set workMvsSpace=E:\viewerout\mvs
echo %piciturePath%
echo %workSpace%
echo %workSfmSpace%
echo %workMvsSpace%

rd /s /q  %workSpace%

mkdir %workSpace%
mkdir %workSfmSpace% 

echo "0. Intrinsics analysis"
D:\repo\OpenMVG.release\openMVG_main_SfMInit_ImageListing -i %piciturePath% -o %workSfmSpace%\matches -d D:\repo\OpenMVG.release\sensor_width_camera_database.txt -f 2000

echo "1. Compute features"
D:\repo\OpenMVG.release\openMVG_main_ComputeFeatures -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches -m SIFT   

echo "2. Compute pairs"
D:\repo\OpenMVG.release\openMVG_main_PairGenerator -i %workSfmSpace%\matches\sfm_data.json -o %workSfmSpace%\matches\pairs.bin
exit

echo "3. Compute matches"
D:\repo\OpenMVG.release\openMVG_main_ComputeMatches -i %workSfmSpace%\matches\sfm_data.json -p %workSfmSpace%\matches\pairs.bin -o %workSfmSpace%\matches\matches.putative.bin -n ANNL2
pause
pause

::echo "4. Filter matches(sequential)"
::D:\repo\OpenMVG.release\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.f.bin
::echo "5. Incremental reconstruction(sequential)"
::D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Release\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s INCREMENTAL  

echo "4. Filter matches(global)"
D:\repo\OpenMVG.release\openMVG_main_GeometricFilter -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches\matches.putative.bin -o %workSfmSpace%\matches\matches.e.bin  -g e
echo "6. Incremental reconstruction(global)"
D:\repo\openMVG\src\build-2019\Windows-AMD64-Release\Release\openMVG_main_SfM -i %workSfmSpace%\matches\sfm_data.json -m %workSfmSpace%\matches -o %workSfmSpace% -s GLOBAL  -M  %workSfmSpace%\matches\matches.e.bin


pause