cmake_minimum_required(VERSION 2.8.3)
project(ADVANCED_ECMS)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

add_compile_options(-std=c++11)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES optimizer
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/optimizer.cpp
   src/VehicleInfo.cpp
   src/ICEMG.cpp
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME} src/example.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)




