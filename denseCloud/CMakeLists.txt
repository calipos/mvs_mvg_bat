cmake_minimum_required (VERSION 2.8)

# 项目信息
project (DenseCloud) 
#file(REMOVE  ${PROJECT_SOURCE_DIR}view.pb.cc) 
#file(REMOVE  ${PROJECT_SOURCE_DIR}view.pb.h)
set(Protobuf_RootDIR protobuf-3.20.0-rc3-shared)
set(Protobuf_INCLUDE_DIR  ${thirdPart}/${Protobuf_RootDIR}/include)
set(Protobuf_LIBRARIES  ${thirdPart}/${Protobuf_RootDIR}/lib)
set(Protobuf_PROTOC_EXECUTABLE  ${thirdPart}/${Protobuf_RootDIR}/bin/protoc.exe)
find_package(Protobuf REQUIRED) 
IF (${Protobuf_FOUND})
    message (STATUS "protobuf found: ${Protobuf_VERSION} ")  
    message (STATUS "protobuf includeDir: ${Protobuf_INCLUDE_DIR}")
    message (STATUS "protobuf libDir: ${Protobuf_LIBRARIES}")
    message (STATUS "Protobuf_PROTOC_EXECUTABLE: ${Protobuf_PROTOC_EXECUTABLE}")
ELSE (${Protobuf_FOUND})
    MESSAGE(FATAL_ERROR "Could not locate GTK.")
ENDIF (${Protobuf_FOUND})



execute_process(
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I=${PROJECT_SOURCE_DIR} --cpp_out=${PROJECT_SOURCE_DIR}  ${PROJECT_SOURCE_DIR}/view.proto
            )
            
include_directories (${thirdPart}/glog_release/include)
include_directories (${thirdPart}/Eigen/eigen3)
include_directories (${thirdPart}/opencv-static/include)
include_directories (${thirdPart}/boost164)
include_directories (${Protobuf_INCLUDE_DIR})
include_directories (${openMvgInstall}Release/include/openMVG_dependencies/cereal/include)
add_executable(DenseCloud denseCloudMain.cpp types.cpp SelectViews.cpp view.pb.cc common.h SelectViews.h types.h utils.h timer.h serialization.h view.pb.h) 
add_definitions(-DNOMINMAX  -D_USE_MATH_DEFINES  -DOPENMVG_USE_AVX2 -DOPENMVG_USE_AVX -DOPENMVG_USE_OPENMP -DEIGEN_MPL2_ONLY -DUSE_PATENTED_LIGT -DLEMON_ONLY_TEMPLATES)
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES  -DGOOGLE_GLOG_DLL_DECL=)
#D_SCL_SECURE_NO_WARNINGS

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "found openmp") 
    message ( STATUS "OpenMP_CXX_FLAGS  = ${OpenMP_CXX_FLAGS}" )
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS}, ${OPENMP_C_FLAGS})
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}, ${OPENMP_CXX_FLAGS})
else()
    message(FATAL_ERROR "openmp not found!")
endif()
if (MSVC)
  add_compile_options("/bigobj")
  add_compile_options("/openmp")
endif(MSVC)
if(NOT OpenCV_SHARED)
foreach(flag_var
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
  if(${flag_var} MATCHES "/MD")
    string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
  endif()
  if(${flag_var} MATCHES "/MDd")
    string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
  endif()
endforeach(flag_var)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
endif()
message("")
message("  C/C++:")
message("    Built as dynamic libs?:" BUILD_SHARED_LIBS THEN YES ELSE NO)
if(DEFINED CMAKE_CXX_STANDARD AND CMAKE_CXX_STANDARD)
  message("    C++ standard:"           "${CMAKE_CXX_STANDARD}")
endif()
message("    C++ Compiler:"           ${OPENCV_COMPILER_STR})
message("    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
message("    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
message("    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
message("    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
message("    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
if(WIN32)
  message("    Linker flags (Release):" ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_RELEASE})
  message("    Linker flags (Debug):"   ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_DEBUG})
else()
  message("    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
  message("    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})
endif()


target_link_libraries ( ${PROJECT_NAME} PRIVATE
						debug ${thirdPart}/opencv-static/lib/opencv_core452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_highgui452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_imgcodecs452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_imgproc452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_aruco452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_calib3d452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_flann452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_features2d452d.lib
                        debug ${thirdPart}/opencv-static/lib/opencv_dnn452d.lib
                        debug ${thirdPart}/opencv-static/lib/libprotobufd.lib
                        debug ${thirdPart}/opencv-static/lib/zlibd.lib
                        debug ${thirdPart}/opencv-static/lib/ittnotifyd.lib
                        debug ${thirdPart}/opencv-static/lib/IlmImfd.lib
                        debug ${thirdPart}/opencv-static/lib/libjpeg-turbod.lib
                        debug ${thirdPart}/opencv-static/lib/libpngd.lib
                        debug ${thirdPart}/opencv-static/lib/libtiffd.lib
                        debug ${thirdPart}/opencv-static/lib/libopenjp2d.lib
                        debug ${thirdPart}/opencv-static/lib/libwebpd.lib
                        debug ${thirdPart}/glog_debug/lib/glog.lib
                        debug ${thirdPart}/boost164/lib/libboost_serialization-vc141-mt-sgd-1_64.lib
                        debug ${Protobuf_LIBRARIES}/libprotobufd.lib
                        #debug ${Protobuf_LIBRARIES}/libprotobuf-lited.lib
                        debug ${Protobuf_LIBRARIES}/libprotocd.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_core452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_highgui452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_imgcodecs452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_imgproc452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_aruco452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_calib3d452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_flann452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_features2d452.lib
                        optimized ${thirdPart}/opencv-static/lib/opencv_dnn452.lib
                        optimized ${thirdPart}/opencv-static/lib/libprotobuf.lib
                        optimized ${thirdPart}/opencv-static/lib/zlib.lib
                        optimized ${thirdPart}/opencv-static/lib/ittnotify.lib
                        optimized ${thirdPart}/opencv-static/lib/IlmImf.lib
                        optimized ${thirdPart}/opencv-static/lib/libjpeg-turbo.lib
                        optimized ${thirdPart}/opencv-static/lib/libpng.lib
                        optimized ${thirdPart}/opencv-static/lib/libtiff.lib
                        optimized ${thirdPart}/opencv-static/lib/libopenjp2.lib
                        optimized ${thirdPart}/opencv-static/lib/libwebp.lib
                        optimized ${thirdPart}/glog_release/lib/glog.lib
                        optimized ${thirdPart}/boost164/lib/libboost_serialization-vc141-mt-s-1_64.lib
                        optimized ${Protobuf_LIBRARIES}/libprotobuf.lib
                        #optimized ${Protobuf_LIBRARIES}/libprotobuf-lite.lib
                        optimized ${Protobuf_LIBRARIES}/libprotoc.lib
                        )
 
#include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include/openMVG_dependencies)
#include_directories (${openMvgInstall}${CMAKE_BUILD_TYPE}/include                                    ) 
#target_link_libraries(DenseCloud ${openMvgInstall}${CMAKE_BUILD_TYPE}/lib/openMVG_features.lib                    ) 

install(TARGETS DenseCloud 
        RUNTIME DESTINATION bin  # 可执行文件安装路径 
        )


 




