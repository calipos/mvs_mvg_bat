cmake_minimum_required (VERSION 2.8)

# 项目信息
project (ReplaceOpenMvgFeature) 
set(openMvgInstall D:/repo/openMVG/src/build-2019/install)


add_executable(ReplaceOpenMvgFeature insertNewMatchMain.cpp)

add_definitions(-DNOMINMAX  -D_USE_MATH_DEFINES  -DOPENMVG_USE_AVX2 -DOPENMVG_USE_AVX -DOPENMVG_USE_OPENMP -DEIGEN_MPL2_ONLY -DUSE_PATENTED_LIGT -DLEMON_ONLY_TEMPLATES)

include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG_dependencies)
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include                                    )
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG                            )
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG_dependencies/cereal/include)
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG/third_party                )
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG/third_party/eigen          )
include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG/third_party/lemon          )





target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_features.lib                    )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_matching_image_collection.lib   )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_multiview.lib                   )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_sfm.lib                         )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_system.lib                      )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_stlplus.lib                     )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_matching.lib                    )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_fast.lib                        )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_geometry.lib                    )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_image.lib                       )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/vlsift.lib                              )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_jpeg.lib                        )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_png.lib                         )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_tiff.lib                        )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_zlib.lib                        )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_lInftyComputerVision.lib        )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_linearProgramming.lib           )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_numeric.lib                     )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/lib_clp.lib                             )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/lib_OsiClpSolver.lib                    )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/lib_CoinUtils.lib                       )
target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/lib_Osi.lib                             )
if (CMAKE_BUILD_TYPE)
    string(TOLOWER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
   
    if (${BUILD_TYPE} STREQUAL "debug")
        target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_ceres-debug.lib          )
    elseif(CMAKE_BUILD_TYPE STREQUAL "release")
        target_link_libraries(ReplaceOpenMvgFeature ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_ceres.lib          )
    else()
        message("set Default CMAKE_BUILD_TYPE : ${CMAKE_BUILD_TYPE}")
    endif()
else()
    set (CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O0")
    message("set Default CMAKE_BUILD_TYPE : ${CMAKE_BUILD_TYPE}")
endif()
install(TARGETS ReplaceOpenMvgFeature 
        RUNTIME DESTINATION bin  # 可执行文件安装路径 
        )


 




