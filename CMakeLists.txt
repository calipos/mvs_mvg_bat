cmake_minimum_required (VERSION 2.8)					# cmake版本最低要求
project (test)
#SET(CMAKE_SYSTEM_NAME Linux)
#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)			# 设置了Debug版本和Release版本可执行文件的输出目录
#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)			# 设置可执行文件的输出目录
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)			# 设置库文件的输出目录

#include_directories (${PROJECT_SOURCE_DIR}/inc)				# 附加头文件目录
option(BUILD_SHARED_LIBS "option for shared" OFF)


set(thirdPart D:/ucl360/libraries2019) 
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/DlibLandmark)				# 先执行这个目录下的cmake
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/replaceOpenMvgFeature)					# 上个目录下生成库后，此时再链接生成可执行文件
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/denseCloud)				 
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/generatePipeMediaFeature)
SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})