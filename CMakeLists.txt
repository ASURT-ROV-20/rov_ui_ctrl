cmake_minimum_required(VERSION 2.8.3)
project(rov_ui_ctrl)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)
find_package(Qt5 COMPONENTS Widgets Network REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_demo
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCE_FILES "*/*.cpp" "*/*.h" "*.cpp" "*.h")
file(GLOB UI "*/*.ui" "./*.ui")

qt5_wrap_ui(UI_HEADERS ${UI})
add_executable(rov_ui_ctrl ${SOURCE_FILES} ${UI_HEADERS})

string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
target_link_libraries(rov_ui_ctrl PRIVATE
    Qt5::Widgets
    Qt5::Network
    ${catkin_LIBRARIES}
    ${SDL2_LIBRARIES}
)

