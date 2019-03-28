cmake_minimum_required(VERSION 3.5)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package (GSL REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_LIBRARY_DIRS}
        "/usr/include/eigen3"
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/data_operator.cpp
        ${PROJECT_SOURCE_DIR}/src/Params.cpp
	${PROJECT_SOURCE_DIR}/src/MuInfo.cpp
)

add_executable(main main.cpp)

target_link_libraries(main
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        opencv_core 
        opencv_highgui
        opencv_flann 
        opencv_imgproc 
        opencv_highgui 
        opencv_ml 
        opencv_video 
        opencv_objdetect
        opencv_features2d 
        opencv_calib3d 
        opencv_legacy 
        opencv_contrib
)
