cmake_minimum_required(VERSION 2.8)
project(StemDetect)
find_package(OpenCV 2.4 REQUIRED)
add_definitions("-std=c++11")
add_executable(stem_detector stem_detector_gui.cpp stem_detector.cpp tic_toc.cpp)
target_link_libraries(stem_detector ${OpenCV_LIBS} )
