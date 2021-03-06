cmake_minimum_required(VERSION 3.5)
project(camera_calibration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)

# dbow3 
# dbow3 is a simple lib so I assume you installed it in default directory 
# set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
# set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3"
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/LoadData.cpp
        ${PROJECT_SOURCE_DIR}/src/KeyPointsMaker.cpp
        ${PROJECT_SOURCE_DIR}/src/PNPSolver.cpp
	# ${PROJECT_SOURCE_DIR}/src/LoopDetector.cpp

)

add_executable(main main.cpp)
target_link_libraries(main
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
	# ${DBoW3_LIBS}
)
