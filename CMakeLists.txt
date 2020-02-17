cmake_minimum_required(VERSION 3.10)
project(mono_calibrator)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations -O3 -pipe")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mono-camera-calibrator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

