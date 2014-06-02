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

## Files that describe classes that are used within the project
add_library(node_class include/NodeClass.cpp)


## Rod camera rgb color calib
add_executable(color_calib src/rod_camera_rgb_color_calib.cpp)
target_link_libraries(color_calib ${catkin_LIBRARIES})
target_link_libraries(color_calib node_class)
add_dependencies(color_calib aod_generate_messages_cpp)


## Rod analysis
add_executable(analysis src/rod_analysis.cpp)
target_link_libraries(analysis ${catkin_LIBRARIES})
add_dependencies(analysis aod_generate_messages_cpp)


## Cod
add_executable(cod src/cod.cpp)
target_link_libraries(cod ${catkin_LIBRARIES})
add_dependencies(cod aod_generate_messages_cpp)


## Rod camera image aggregation
add_executable(image_aggregation src/rod_camera_image_aggregation.cpp)
target_link_libraries(image_aggregation ${catkin_LIBRARIES})
target_link_libraries(image_aggregation node_class)
add_dependencies(image_aggregation aod_generate_messages_cpp)


## Rod camera rgb stfp
add_executable(rgb_stfp src/rod_camera_rgb_stfp.cpp)
target_link_libraries(rgb_stfp ${catkin_LIBRARIES})
target_link_libraries(rgb_stfp node_class)
add_dependencies(rgb_stfp aod_generate_messages_cpp)


## Rod camera ir stfp
add_executable(ir_stfp src/rod_camera_ir_stfp.cpp)
target_link_libraries(ir_stfp ${catkin_LIBRARIES})
target_link_libraries(ir_stfp node_class)
add_dependencies(ir_stfp aod_generate_messages_cpp)


## Rod camera ir image proc
add_executable(ir_image_proc src/rod_camera_ir_image_proc.cpp)
target_link_libraries(ir_image_proc ${catkin_LIBRARIES})
target_link_libraries(ir_image_proc node_class)
add_dependencies(ir_image_proc aod_generate_messages_cpp)

# %EndTag(FULLTEXT)%