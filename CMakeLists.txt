# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(aod)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sorter_msgs image_proc sensor_msgs dynamic_reconfigure cv_bridge image_transport polled_camera)
find_package(OpenCV)

## Generate dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/cod.cfg
  cfg/analysis.cfg
  cfg/color_calib.cfg
  cfg/image_aggregation.cfg
)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs sorter_msgs sensor_msgs )

add_definitions("-std=c++11")
add_definitions("-Wno-write-strings")

## Declare a catkin package
catkin_package(
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


## Rod camera rgb color calib
add_executable(color_calib src/rod_camera_rgb_color_calib.cpp)
add_dependencies(color_calib color_calib_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_library(ColorCalibrationDll src/color_calibration/ColorCalibrationDll.cpp)
target_link_libraries(color_calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ColorCalibrationDll)


## Rod analysis
add_executable(analysis src/rod_analysis.cpp
	src/STEM/stem_detector.cpp
	src/STEM/stem_detector_main.cpp
	src/STEM/tic_toc.cpp)
add_dependencies(analysis analysis_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(analysis ${catkin_LIBRARIES})


## Cod
add_executable(cod src/cod.cpp)
add_dependencies(cod cod ${PROJECT_NAME}_gencfg)
target_link_libraries(cod ${catkin_LIBRARIES})


## Rod camera image aggregation
add_executable(image_aggregation src/rod_camera_image_aggregation.cpp)
add_dependencies(image_aggregation image_aggregation_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(image_aggregation ${catkin_LIBRARIES})

add_executable(sync src/sync.cpp)
target_link_libraries(sync ${catkin_LIBRARIES})

#add_executable(rod 
#	src/main.cpp 
#	src/STEM/stem_detector.cpp
#	src/STEM/stem_detector_main.cpp
#	src/STEM/tic_toc.cpp
#)
#target_link_libraries(rod ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# %EndTag(FULLTEXT)%
