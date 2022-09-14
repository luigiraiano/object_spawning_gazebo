cmake_minimum_required(VERSION 3.0.2)
project(object_spawning_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    roscpp
    rospy
    std_msgs
    tf
    xacro
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES object_spawning_gazebo
    CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs tf
    DEPENDS gazebo_ros roscpp rospy std_msgs tf xacro
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)

# Sphere Shooter
add_library(${PROJECT_NAME}_sphere_shooter src/sphere_shooter_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_sphere_shooter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Ball Shooter
add_library(${PROJECT_NAME}_ball_shooter_plugin
    src/BallShooterPlugin.cpp
)
target_link_libraries(${PROJECT_NAME}_ball_shooter_plugin PUBLIC
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

## Target Pose Publisher Node
add_executable(${PROJECT_NAME}_target_pose_pub_node src/target_pose_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_target_pose_pub_node PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_target_pose_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
