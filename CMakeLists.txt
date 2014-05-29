# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(aod)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(color_calib rod_camera_rgb_color_calib.cpp)
target_link_libraries(color_calib ${catkin_LIBRARIES})
add_dependencies(color_calib aod_generate_messages_cpp)

add_executable(analysis rod_analysis.cpp)
target_link_libraries(analysis ${catkin_LIBRARIES})
add_dependencies(analysis aod_generate_messages_cpp)

add_executable(cod cod.cpp)
target_link_libraries(cod ${catkin_LIBRARIES})
add_dependencies(cod aod_generate_messages_cpp)

add_executable(image_aggregation rod_camera_image_aggregation.cpp)
target_link_libraries(image_aggregation ${catkin_LIBRARIES})
add_dependencies(image_aggregation aod_generate_messages_cpp)

add_executable(stfp rod_camera_rgb_stfp.cpp)
target_link_libraries(stfp ${catkin_LIBRARIES})
add_dependencies(stfp aod_generate_messages_cpp)

add_executable(ir_image_proc rod_camera_ir_image_proc.cpp)
target_link_libraries(ir_image_proc ${catkin_LIBRARIES})
add_dependencies(ir_image_proc aod_generate_messages_cpp)

# %EndTag(FULLTEXT)%