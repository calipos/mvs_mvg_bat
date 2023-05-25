cmake_minimum_required (VERSION 2.8)

# 项目信息
project (DlibLandmark)
set(thirdPart D:/ucl360/libraries2019) 
set (CMAKE_BUILD_TYPE "Release")
add_executable(DlibLandmark DlibLandmark.cpp)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd") 
 
#SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/..)
#set_target_properties( DlibLandmark
#    PROPERTIES
#    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
#    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
#)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR}/..)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR}/..)
SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/..)
install(TARGETS DlibLandmark 
        RUNTIME DESTINATION bin  # 可执行文件安装路径 
        )
        

include_directories (${thirdPart}/dlib1924/include)
include_directories (${thirdPart}/glog_release/include)
include_directories (${thirdPart})
 
 
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES  -DGOOGLE_GLOG_DLL_DECL=)
target_link_libraries(DlibLandmark ${thirdPart}/dlib1924/lib/dlib19.24.0_release_64bit_msvc1929.lib)
target_link_libraries(DlibLandmark ${thirdPart}/glog_release/lib/glog.lib)











