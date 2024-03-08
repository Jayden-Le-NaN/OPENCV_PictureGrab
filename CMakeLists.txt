cmake_minimum_required(VERSION 3.12)
project(DataSetCollection)

#------------------------------basic setting------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 20)
set(MVCAM_COMMON_RUNENV /opt/MVS/lib)
set(MVCAM_COMMON_PATH /opt/MVS)

file(GLOB HPP_FILES "${PROJECT_SOURCE_DIR}/inc/*.hpp")
file(GLOB CPP_FILES "${PROJECT_SOURCE_DIR}/inc/*.cpp")

#------------------------------inlcude head file------------------------------
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/inc)

add_executable(app 
    main.cpp
    ${HPP_FILES}
)

target_include_directories(app PRIVATE ${MVCAM_COMMON_PATH}/include)
target_link_directories(app PRIVATE ${MVCAM_COMMON_RUNENV}/aarch64)
target_link_libraries(app PRIVATE ${OpenCV_LIBS} ${realsense2_LIBRARY} MvCameraControl jpeg png yaml-cpp)

