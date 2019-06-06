cmake_minimum_required(VERSION 2.8.3)
project(ecms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

add_compile_options(-std=c++14)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}_lib
CATKIN_DEPENDS std_msgs geometry_msgs
#DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  src
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ecms_lib
   src/optimizer.cpp
   src/VehicleInfo.cpp
   src/interp_tool.cpp
   src/Tool.cpp
   src/constraints.cpp
   src/ICEMG.cpp
   src/performance.cpp
   src/data_loader.cpp
)

target_link_libraries(ecms_lib m pthread stdc++ ${catkin_LIBRARIES})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ecms src/example.cpp)

target_link_libraries(ecms ecms_lib)


install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)





