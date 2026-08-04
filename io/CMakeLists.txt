cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera_config.cpp
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
set(HIKROBOT_MVS_LIBRARY "" CACHE FILEPATH "Path to libMvCameraControl.so")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  if(NOT HIKROBOT_MVS_LIBRARY)
    if(EXISTS "/opt/MVS/lib/64/libMvCameraControl.so")
      set(HIKROBOT_MVS_LIBRARY "/opt/MVS/lib/64/libMvCameraControl.so")
    else()
      set(HIKROBOT_MVS_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64/libMvCameraControl.so")
    endif()
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  if(NOT HIKROBOT_MVS_LIBRARY)
    set(HIKROBOT_MVS_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMvCameraControl.so")
  endif()
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()
if(NOT EXISTS "${HIKROBOT_MVS_LIBRARY}")
  message(FATAL_ERROR "Hikrobot MVS library not found: ${HIKROBOT_MVS_LIBRARY}")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io ${HIKROBOT_MVS_LIBRARY} MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT SRM_VISION_ENABLE_ROS2 OR NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs)
endif()
