cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER   "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(sfm)
#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Add libraries
find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.5    REQUIRED common io visualization)
find_package(OpenCV     REQUIRED core highgui imgproc xfeatures2d xphoto)
find_package(Boost      REQUIRED program_options system filesystem date_time)
find_package(Ceres      REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")
include_directories("/usr/local/include/eigen3/")
include_directories(${CERES_INCLUDE_DIRS})
#set(Boost_USE_STATIC_LIBS ON)

add_definitions(${PCL_DEFINITIONS})

# OS X specific options
if (APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -stdlib=libc++")
  set(CMAKE_CXX_LINK_FLAGS "-lc++")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -std=c++11")
endif(APPLE)

# Set compile options
set(
  CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -g -O3 -msse3 -msse4 -mavx"
)

# Add libraries
add_subdirectory(src)

# Main binary
add_executable(main "src/main.cpp")

# Depend on OpenCV and Boost
target_link_libraries(main pipeline ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CERES_LIBRARIES} )

