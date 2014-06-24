# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(aod)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sorter_msgs image_proc sensor_msgs dynamic_reconfigure cv_bridge)

## Generate dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/cod.cfg
  cfg/analysis.cfg
  cfg/color_calib.cfg
  cfg/image_aggregation.cfg
)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs sorter_msgs sensor_msgs )

## Declare a catkin package
catkin_package(
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


## Rod camera rgb color calib
add_executable(color_calib src/rod_camera_rgb_color_calib.cpp)
add_dependencies(color_calib color_calib_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(color_calib ${catkin_LIBRARIES})
add_library(ColorCalibrationDll src/color_calibration/ColorCalibrationDll.cpp)


## Rod analysis
add_executable(analysis src/rod_analysis.cpp)
add_dependencies(analysis analysis_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(analysis ${catkin_LIBRARIES})


## Cod
add_executable(cod src/cod.cpp)
add_dependencies(cod cod ${PROJECT_NAME}_gencfg)
target_link_libraries(cod ${catkin_LIBRARIES})


## Rod camera image aggregation
add_executable(image_aggregation src/rod_camera_image_aggregation.cpp)
add_dependencies(image_aggregation image_aggregation ${PROJECT_NAME}_gencfg)
target_link_libraries(image_aggregation ${catkin_LIBRARIES})



# %EndTag(FULLTEXT)%
