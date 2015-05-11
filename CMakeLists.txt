cmake_minimum_required(VERSION 2.8)

set(
  CMAKE_CXX_COMPILER
  "clang++"
)

project(sfm)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Add libraries
find_package(OpenCV REQUIRED core highgui imgproc xfeatures2d xphoto)
find_package(Boost  REQUIRED program_options system filesystem date_time)
find_package(Ceres  REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
#set(Boost_USE_STATIC_LIBS ON)

# OS X specific options
if (APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -stdlib=libc++")
  set(CMAKE_CXX_LINK_FLAGS "-lc++")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(APPLE)

# Set compile options
set(
  CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -g -O0 -msse3 -msse4"
)

# Add libraries
add_subdirectory(src)

# Main binary
add_executable(main "src/main.cpp")

# Depend on OpenCV and Boost
target_link_libraries(main pipeline ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Ceres_LIBS} )