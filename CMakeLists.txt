cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

project(exploration LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include)

# # RRT EXAMPLE
# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

add_executable(simple_rrt
  src/rrt.cpp
  src/simple_rrt_plot.cpp)

target_include_directories(simple_rrt PUBLIC ${PYTHON_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

target_link_libraries(simple_rrt PRIVATE
  ${PYTHON_LIBRARIES}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS})