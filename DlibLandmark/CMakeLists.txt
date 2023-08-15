cmake_minimum_required (VERSION 2.8)

# 项目信息
project (DlibLandmark)

set (CMAKE_BUILD_TYPE "Release")
add_executable(DlibLandmark DlibLandmark.cpp)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd") 



install(TARGETS DlibLandmark 
        RUNTIME DESTINATION bin  # 可执行文件安装路径 
        )

add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES  -DGOOGLE_GLOG_DLL_DECL=)
include_directories (${thirdPart})
include_directories (${thirdPart}/dlib1924/include) 
include_directories (${thirdPart}/glog_release/include) 
target_link_libraries ( ${PROJECT_NAME} PRIVATE
						debug ${thirdPart}/dlib1924/lib/dlib19.24.0_debug_64bit_msvc1929.lib
						debug ${thirdPart}/glog_debug/lib/glog.lib
						optimized ${thirdPart}/dlib1924/lib/dlib19.24.0_release_64bit_msvc1929.lib
						optimized ${thirdPart}/glog_release/lib/glog.lib)











