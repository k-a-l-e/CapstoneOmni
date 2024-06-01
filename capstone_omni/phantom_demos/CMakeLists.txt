cmake_minimum_required(VERSION 3.0.2)
project(phantom_demos)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs 
  tf2_geometry_msgs
  omni_msgs 
  sensor_msgs 
  std_msgs 
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  # This should include ROS headers
)

catkin_package(
CATKIN_DEPENDS 
  geometry_msgs 
  omni_description 
  omni_msgs 
  robot_state_publisher 
  sensor_msgs 
  std_msgs 
  tf
  xacro
)

include_directories(${OPENHAPTICS_INCLUDE_DIR})
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}") 
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)

set(NODE_1 demo)
set(NODE_2 phantom_node)
set(NODE_3 scale_tf_node)
set(NODE_4 spring_node)

add_executable(${NODE_1} src/demo.cpp)
add_dependencies(${NODE_1} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_1} HD HDU rt ncurses ${catkin_LIBRARIES})
target_include_directories(${NODE_1} PUBLIC ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include)

add_executable(${NODE_2} src/phantom_node.cpp)
add_dependencies(${NODE_2} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_2} HD HDU rt ncurses ${catkin_LIBRARIES})
target_include_directories(${NODE_2} PUBLIC ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include)

add_executable(${NODE_3} src/scale_tf_node.cpp)
add_dependencies(${NODE_3} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_3} ${catkin_LIBRARIES})

add_executable(${NODE_4} src/spring_node.cpp)
add_dependencies(${NODE_4} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_4} HD HDU ${catkin_LIBRARIES})
