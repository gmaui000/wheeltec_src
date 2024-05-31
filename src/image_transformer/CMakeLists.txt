cmake_minimum_required(VERSION 3.5)
project(image_transformer)

find_package(ament_cmake_ros REQUIRED)
## pkg-config libraries
find_package(PkgConfig REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/image_transform.cpp)

ament_target_dependencies(${PROJECT_NAME}
  cv_bridge image_transport rclcpp sensor_msgs)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_package()
