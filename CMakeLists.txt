cmake_minimum_required(VERSION 3.24)
project(pcl-examples)

#
# PCL
#
if(WIN32)
	add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
	find_path(PCL_DIR PCLConfig.cmake PATHS 
		$ENV{PCL_DIR}
		"D:/install/PCL 1.13.0"
	)
endif()
find_package(PCL REQUIRED)

# 引入PCL的方法一：用PCL自带的变量引用即可
MESSAGE(STATUS "[PCL_INCLUDE_DIRS] ${PCL_INCLUDE_DIRS}")
MESSAGE(STATUS "[PCL_LIBRARY_DIRS] ${PCL_LIBRARY_DIRS}")
MESSAGE(STATUS "[PCL_LIBRARIES] ${PCL_LIBRARIES}")
MESSAGE(STATUS "[PCL_DEFINITIONS] ${PCL_DEFINITIONS}")

# 引入PCL的方法二：自己创建一个Target，方便子工程链接
add_library(PCL INTERFACE)
target_include_directories(PCL INTERFACE ${PCL_INCLUDE_DIRS})
target_link_directories(PCL INTERFACE ${PCL_LIBRARY_DIRS})
target_link_libraries(PCL INTERFACE ${PCL_LIBRARIES})
# add_compile_definitions(PCL INTERFACE ${PCL_DEFINITIONS})

#
# Boost
#
find_package(Boost REQUIRED COMPONENTS thread locale)

#
# 路径
#

set(DataDir "${CMAKE_SOURCE_DIR}/data/")

#工程公共头文件
set(CommonIncludeDir ${CMAKE_SOURCE_DIR}/include)  #工程头文件目录
include_directories(${CommonIncludeDir})           #添加include目录（全局）
file(GLOB_RECURSE CommonHeadersFiles ${CommonIncludeDir}/*.h ${CommonIncludeDir}/*.hpp)

#
# 全局置为C++17
#   https://cmake.org/cmake/help/latest/prop_tgt/CXX_STANDARD.html
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#
# 子工程
#
add_subdirectory(examples)

#
# 其他配置
#

# 代码分组
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${CommonHeadersFiles})
# 开启工程分组
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
# 关闭编译器优化
set(CMAKE_CXX_FLAGS "-fno-elide-constructors ${CMAKE_CXX_FLAGS}")