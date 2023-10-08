cmake_minimum_required (VERSION 2.8)

# 项目信息
project (ParseMVG) 

add_executable(ParseMVG parseMVG.cpp ) 

add_definitions(-DNOMINMAX  -D_USE_MATH_DEFINES  -DOPENMVG_USE_AVX2 -DOPENMVG_USE_AVX -DOPENMVG_USE_OPENMP -DEIGEN_MPL2_ONLY -DUSE_PATENTED_LIGT -DLEMON_ONLY_TEMPLATES)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "found openmp") 
    message ( STATUS "OpenMP_CXX_FLAGS  = ${OpenMP_CXX_FLAGS}" )
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS}, ${OPENMP_C_FLAGS})
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}, ${OPENMP_CXX_FLAGS})
else()
    message(FATAL_ERROR "openmp not found!")
endif()



 
include_directories (${openMvgInstall}Release/include/openMVG_dependencies)
include_directories (${openMvgInstall}Release/include                                    )
include_directories (${openMvgInstall}Release/include/openMVG                            )
include_directories (${openMvgInstall}Release/include/openMVG_dependencies/cereal/include)
include_directories (${openMvgInstall}Release/include/openMVG/third_party                )
include_directories (${openMvgInstall}Release/include/openMVG/third_party/eigen          )
include_directories (${openMvgInstall}Release/include/openMVG/third_party/lemon          )



target_link_libraries ( ${PROJECT_NAME} PRIVATE
						debug ${openMvgInstall}Debug/lib/openMVG_ceres-debug.lib
						debug ${openMvgInstall}Debug/lib/openMVG_features.lib                    
						debug ${openMvgInstall}Debug/lib/openMVG_matching_image_collection.lib   
						debug ${openMvgInstall}Debug/lib/openMVG_multiview.lib                   
						debug ${openMvgInstall}Debug/lib/openMVG_sfm.lib                         
						debug ${openMvgInstall}Debug/lib/openMVG_system.lib                      
						debug ${openMvgInstall}Debug/lib/openMVG_stlplus.lib                     
						debug ${openMvgInstall}Debug/lib/openMVG_matching.lib                    
						debug ${openMvgInstall}Debug/lib/openMVG_fast.lib                        
						debug ${openMvgInstall}Debug/lib/openMVG_geometry.lib                    
						debug ${openMvgInstall}Debug/lib/openMVG_image.lib                       
						debug ${openMvgInstall}Debug/lib/vlsift.lib                              
						debug ${openMvgInstall}Debug/lib/openMVG_jpeg.lib                        
						debug ${openMvgInstall}Debug/lib/openMVG_png.lib                         
						debug ${openMvgInstall}Debug/lib/openMVG_tiff.lib                        
						debug ${openMvgInstall}Debug/lib/openMVG_zlib.lib                        
						debug ${openMvgInstall}Debug/lib/openMVG_lInftyComputerVision.lib        
						debug ${openMvgInstall}Debug/lib/openMVG_linearProgramming.lib           
						debug ${openMvgInstall}Debug/lib/openMVG_numeric.lib                     
						debug ${openMvgInstall}Debug/lib/lib_clp.lib                             
						debug ${openMvgInstall}Debug/lib/lib_OsiClpSolver.lib                    
						debug ${openMvgInstall}Debug/lib/lib_CoinUtils.lib                       
						debug ${openMvgInstall}Debug/lib/lib_Osi.lib            
                        optimized ${openMvgInstall}Release/lib/openMVG_ceres.lib
						optimized ${openMvgInstall}Release/lib/openMVG_features.lib                    
						optimized ${openMvgInstall}Release/lib/openMVG_matching_image_collection.lib   
						optimized ${openMvgInstall}Release/lib/openMVG_multiview.lib                   
						optimized ${openMvgInstall}Release/lib/openMVG_sfm.lib                         
						optimized ${openMvgInstall}Release/lib/openMVG_system.lib                      
						optimized ${openMvgInstall}Release/lib/openMVG_stlplus.lib                     
						optimized ${openMvgInstall}Release/lib/openMVG_matching.lib                    
						optimized ${openMvgInstall}Release/lib/openMVG_fast.lib                        
						optimized ${openMvgInstall}Release/lib/openMVG_geometry.lib                    
						optimized ${openMvgInstall}Release/lib/openMVG_image.lib                       
						optimized ${openMvgInstall}Release/lib/vlsift.lib                              
						optimized ${openMvgInstall}Release/lib/openMVG_jpeg.lib                        
						optimized ${openMvgInstall}Release/lib/openMVG_png.lib                         
						optimized ${openMvgInstall}Release/lib/openMVG_tiff.lib                        
						optimized ${openMvgInstall}Release/lib/openMVG_zlib.lib                        
						optimized ${openMvgInstall}Release/lib/openMVG_lInftyComputerVision.lib        
						optimized ${openMvgInstall}Release/lib/openMVG_linearProgramming.lib           
						optimized ${openMvgInstall}Release/lib/openMVG_numeric.lib                     
						optimized ${openMvgInstall}Release/lib/lib_clp.lib                             
						optimized ${openMvgInstall}Release/lib/lib_OsiClpSolver.lib                    
						optimized ${openMvgInstall}Release/lib/lib_CoinUtils.lib                       
						optimized ${openMvgInstall}Release/lib/lib_Osi.lib     
						)
 

install(TARGETS ParseMVG 
        RUNTIME DESTINATION bin  # 可执行文件安装路径 
        )


 




