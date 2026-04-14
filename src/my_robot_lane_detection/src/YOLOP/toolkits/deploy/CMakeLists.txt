cmake_minimum_required(VERSION 3.10)

project(yolop)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/cuda-10.2/include
    /usr/include/aarch64-linux-gnu/
    /usr/local/zed/include
)

link_directories(
    /usr/local/cuda-10.2/lib64
    /usr/lib/aarch64-linux-gnu/
    /usr/local/zed/lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

set(ZED_LIBS
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    ${CUDA_CUDART_LIBRARY}
)

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

add_executable(yolop ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(yolop
    nvinfer
    ${ZED_LIBS}
    cudart
    myplugins
    ${OpenCV_LIBS}
)

add_definitions(-O3 -pthread)
