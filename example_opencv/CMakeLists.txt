cmake_minimum_required(VERSION 2.8.3)
project(example_opencv)

find_package(OpenCV 2 REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "Using OpenCV version ${OpenCV_VERSION} installed at: ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV supported compute capabilities are: ${OpenCV_COMPUTE_CAPABILITIES}")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
  eigen_conversions
  #cv_bridge
  image_transport
  geometry_msgs
  std_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(SOURCE_DIR src)

### Naive blob detector ###
#find_red_pixels src/find_red_pixels.cpp
add_executable(find_red_pixels
  ${SOURCE_DIR}/find_red_pixels.cpp
)

target_link_libraries(find_red_pixels ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

