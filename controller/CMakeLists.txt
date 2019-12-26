cmake_minimum_required(VERSION 2.8.3)
project(ur10e)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  ur_dashboard_msgs
  ur10e_messages
)

find_package(
  Eigen3 REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES linear
#  CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



file(GLOB SRCFILES src/*.cpp)
add_executable(ur10e ${SRCFILES})
target_link_libraries(ur10e ${catkin_LIBRARIES})

#add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
#target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

#add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
#target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

#add_executable(turtle_teleop_joy src/turtle_teleop_joy.cpp)
#target_link_libraries(turtle_teleop_joy ${catkin_LIBRARIES})
