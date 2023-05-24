cmake_minimum_required (VERSION 2.8)

# 项目信息
project (ReplaceOpenMvgFeature)
set(openMVGSrc D:/repo/openMVG/src)
set(openMVGBuild D:/repo/openMVG/build-vs2019)
add_executable(ReplaceOpenMvgFeature insertNewMatchMain.cpp)

add_definitions(-DNOMINMAX  -D_USE_MATH_DEFINES  -DOPENMVG_USE_AVX2 -DOPENMVG_USE_AVX -DOPENMVG_USE_OPENMP -DEIGEN_MPL2_ONLY -DUSE_PATENTED_LIGT -DLEMON_ONLY_TEMPLATES)

include_directories (${openMVGSrc}/third_party/lemon)
include_directories (${openMVGBuild}/third_party/lemon)
include_directories (D:/ucl360/libraries2019/Eigen/eigen3)
include_directories (${openMVGSrc})
include_directories (${openMVGSrc}/dependencies/cereal/include)
include_directories (${openMVGSrc}/third_party/stlplus3/filesystemSimplified)
include_directories (${openMVGBuild}/third_party/ceres-solver/config)
include_directories (${openMVGSrc}/third_party/ceres-solver/include)
include_directories (${openMVGSrc}/third_party/ceres-solver/internal/ceres/miniglog) 

#openMVG_features.lib
#openMVG_matching_image_collection.lib
#openMVG_multiview.lib
#openMVG_sfm.lib
#openMVG_system.lib
#openMVG_stlplus.lib
#openMVG_matching.lib
#openMVG_fast.lib
#openMVG_geometry.lib
#openMVG_image.lib
#vlsift.lib
#openMVG_jpeg.lib
#openMVG_png.lib
#openMVG_tiff.lib
#openMVG_zlib.lib
#openMVG_lInftyComputerVision.lib
#D:/repo/openMVG/src/build-2019/third_party/ceres-solver/lib/Debug/openMVG_ceres-debug.lib
#openMVG_linearProgramming.lib
#openMVG_numeric.lib
#lib_clp.lib
#lib_OsiClpSolver.lib
#lib_CoinUtils.lib
#lib_Osi.lib
#kernel32.lib;user32.lib;gdi32.lib;winspool.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;comdlg32.lib;advapi32.lib
message(${openMVGBuild}/Windows-AMD64-Release/Release)
link_libraries(${openMVGBuild}/Windows-AMD64-Release/Release)


target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_features.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_matching_image_collection.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_multiview.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_sfm.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_system.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_stlplus.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_matching.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_fast.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_geometry.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_image.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/vlsift.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_jpeg.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_png.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_tiff.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_zlib.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_lInftyComputerVision.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_linearProgramming.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/openMVG_numeric.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/lib_clp.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/lib_OsiClpSolver.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/lib_CoinUtils.lib)
target_link_libraries(ReplaceOpenMvgFeature ${openMVGBuild}/Windows-AMD64-Release/Release/lib_Osi.lib)

















