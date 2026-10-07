cmake_minimum_required(VERSION 3.16.3)

# Also build without OpenVINO, camera SDKs, ROS2 or a gimbal:
# cmake -S calibration -B build/uvc_calibration
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(uvc_calibration LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 17)
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio highgui)
    find_package(fmt REQUIRED)
    find_package(yaml-cpp REQUIRED)
endif()

add_library(uvc_calibration STATIC uvc_calibration.cpp)
target_include_directories(uvc_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(uvc_calibration PUBLIC ${OpenCV_LIBS} yaml-cpp)

add_executable(capture_uvc capture_uvc.cpp)
target_link_libraries(capture_uvc PRIVATE uvc_calibration fmt::fmt)

add_executable(calibrate_uvc calibrate_uvc.cpp)
target_link_libraries(calibrate_uvc PRIVATE uvc_calibration fmt::fmt)

add_executable(uvc_calibration_test ../tests/uvc_calibration_test.cpp)
target_link_libraries(uvc_calibration_test PRIVATE uvc_calibration fmt::fmt)

# Match the other repository executables when built from the root.
set_target_properties(capture_uvc calibrate_uvc uvc_calibration_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
