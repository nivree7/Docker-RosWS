cmake_minimum_required(VERSION 3.10)

project(yolop)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

message(STATUS "CUDA version: ${CUDA_VERSION}")
message(STATUS "CUDA include dirs: ${CUDA_INCLUDE_DIRS}")
message(STATUS "CUDA toolkit root: ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "ZED libraries: ${ZED_LIBRARIES}")

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /usr/local/zed/include
)

link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/targets/x86_64-linux/lib
    /usr/local/zed/lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

set(ZED_LIBS
    ${ZED_LIBRARIES}
    ${CUDA_CUDART_LIBRARY}
)

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/yololayer.cu)
target_link_libraries(myplugins
    nvinfer
    ${CUDA_CUDART_LIBRARY}
)

add_executable(yolop ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(yolop
    nvinfer
    ${ZED_LIBS}
    myplugins
    ${OpenCV_LIBS}
)

add_definitions(-O3 -pthread)

install(TARGETS myplugins yolop
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
