cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ExtractClouds)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -fopenmp")
# set(PCL_DIR /mnt/lustre/share/yangguorun/pcl-1.8/share/pcl-1.8)
# set(PCL_DIR /usr/local/pcl-1.8/share/pcl-1.8)
set(LiDAR_SRC
    VelodyneReader.cpp

)

find_package(PCL 1.7 REQUIRED)
set(PCAP_LIB pcap)
set(GLOG_LIB glog)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (Extractpcd Extractpcd.cpp ${LiDAR_SRC} util/TimeUtil.cpp)
target_link_libraries (Extractpcd ${PCL_LIBRARIES} ${PCAP_LIB} ${GLOG_LIB} util)
