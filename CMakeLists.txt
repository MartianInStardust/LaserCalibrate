cmake_minimum_required(VERSION 3.19)
project(LaserCalibrate)

#set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)
find_package(PCL 1.9 REQUIRED)
if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL not FOUND")
endif()

# find_package(PCL 1.9 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions( ${PCL_DEFINITIONS} )


add_library(pcl_process STATIC
        ${PROJECT_SOURCE_DIR}/include/CloudPointExaction.h
        ${PROJECT_SOURCE_DIR}/include/Utils.h
        ${PROJECT_SOURCE_DIR}/src/CloudPointExaction.cpp
        ${PROJECT_SOURCE_DIR}/src/Utils.cpp
        )

target_link_libraries(pcl_process ${PCL_LIBRARIES})

add_executable(LaserCalibrate main.cpp)
target_link_libraries(LaserCalibrate
        pcl_process
        ${PCL_LIBRARIES}
        )
add_executable(test example/test.cpp)
target_link_libraries(test
        pcl_process
        ${PCL_LIBRARIES}
        )

add_executable(TestRegistration example/TestRegistration.cpp)
target_link_libraries(TestRegistration
        pcl_process
        ${PCL_LIBRARIES}
        )

add_executable(DownSample example/DownSample.cpp)
target_link_libraries(DownSample
        pcl_process
        ${PCL_LIBRARIES}
        )