cmake_minimum_required(VERSION 3.0.2)
project(publish_b1_tf)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  unitree_legged_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_b1_tf
#  CATKIN_DEPENDS roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

# include unitree complied .so shared library
link_directories(${CMAKE_SOURCE_DIR}/unitree_ros_to_real/unitree_legged_sdk/lib/cpp/${ARCH})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_ros_to_real/unitree_legged_sdk/include
)

add_executable(publish src/publish.cpp)

target_link_libraries(publish
 libunitree_legged_sdk.so
 ${catkin_LIBRARIES}
)

add_dependencies(publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})