cmake_minimum_required(VERSION 2.8.3)
project(ros_jwwangchn_move)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg mavros_msgs geometry_msgs)


generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# add_dependencies(${PROJECT_NAME} ros_jwwangchn_draw_square_generate_messages_cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ros_jwwangchn_move_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

