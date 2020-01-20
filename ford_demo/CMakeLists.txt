cmake_minimum_required(VERSION 2.8.3)
project(ford_demo)

find_package(catkin REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

