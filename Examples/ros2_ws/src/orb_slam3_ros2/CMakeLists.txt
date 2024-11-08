cmake_minimum_required(VERSION 3.5)

project(orb_slam3_ros2)

# Use modern CMake for better compatibility with ROS2
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define base path for ORB_SLAM3 to avoid long relative paths
set(ORB_SLAM3_PATH "/ORB_SLAM3")

# Include modules
list(APPEND CMAKE_MODULE_PATH ${ORB_SLAM3_PATH}/cmake_modules)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.0 not found.")
    endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(cv_bridge REQUIRED)

# Add include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${ORB_SLAM3_PATH}/include
    ${ORB_SLAM3_PATH}/include/CameraModels
    ${ORB_SLAM3_PATH}/Thirdparty/DBoW2
    ${ORB_SLAM3_PATH}/Thirdparty/DBoW2/DBoW2  # Add the include path for DBoW2
    ${ORB_SLAM3_PATH}/Thirdparty/g2o  # Add the include path for g2o if needed
    ${ORB_SLAM3_PATH}/Thirdparty/Sophus
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Set compilation flags
add_compile_options(-Wall -O3 -march=native)

# Add library paths
link_directories(
    ${ORB_SLAM3_PATH}/lib
    ${ORB_SLAM3_PATH}/Thirdparty/DBoW2/lib
    ${ORB_SLAM3_PATH}/Thirdparty/g2o/lib
)

# Executable for monocular camera
add_executable(MonoNode src/ros2_mono.cpp)
ament_target_dependencies(MonoNode rclcpp sensor_msgs cv_bridge OpenCV Eigen3 Pangolin)

# Libraries to link against
set(LIBS 
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ORB_SLAM3
    DBoW2
    g2o
    -lboost_system
)

# Link libraries to the executable
target_link_libraries(MonoNode ${LIBS})

# Set RPATH to locate shared libraries during runtime
set_target_properties(MonoNode PROPERTIES INSTALL_RPATH ${ORB_SLAM3_PATH}/lib)

# Install targets
install(TARGETS MonoNode
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Package
ament_package()


