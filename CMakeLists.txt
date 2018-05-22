project(transform_point_cloud)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    image_transport
    tf
    )
find_package(OpenCV REQUIRED)


include_directories(include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

catkin_package(
   CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        message_runtime
        tf
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pct src/pct.cpp)
target_link_libraries(pct ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} opencv_core opencv_highgui)
