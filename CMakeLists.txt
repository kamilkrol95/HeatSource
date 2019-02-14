cmake_minimum_required(VERSION 2.8.3)
project(heat_source)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

add_executable(heat_source src/HeatSource.cpp)
add_dependencies(heat_source ${catkin_EXPORTED_TARGETS})
target_link_libraries(heat_source ${OpenCV_LIBS})
target_link_libraries(heat_source
   ${catkin_LIBRARIES} 
   ${opencv2_LIBRARIES} 
   ${naoeus_LIBRARIES}
 )
include_directories(${catkin_INCLUDE_DIRS})
include_directories(/opt/ros/kinetic/include)
link_directories(/opt/ros/kinetic/lib)
