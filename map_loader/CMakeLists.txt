cmake_minimum_required(VERSION 2.8.3)
project(map_loader)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
)
find_package(PCL 1.7 REQUIRED COMPONENTS io)

pkg_check_modules(EIGEN3 REQUIRED eigen3)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
   CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(reflectivity_map_loader src/reflectivity_map_loader.cpp)
add_executable(point_cloud_map_loader src/point_cloud_map_loader.cpp)

target_link_libraries(reflectivity_map_loader ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(point_cloud_map_loader ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES})
