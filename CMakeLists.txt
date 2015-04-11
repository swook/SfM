cmake_minimum_required(VERSION 2.8)

set(
  CMAKE_CXX_COMPILER
  "clang++"
)

project(sfm)

# Add libraries
find_package(OpenCV REQUIRED core highgui imgproc xfeatures2d)
find_package(Boost  REQUIRED program_options system filesystem date_time)

include_directories(${Boost_INCLUDE_DIRS})
#set(Boost_USE_STATIC_LIBS ON)

# Set compile options
set(
  CMAKE_CXX_FLAGS
  ${CMAKE_CXX_FLAGS}
  "-O3 -stdlib=libstdc++ -msse3 -msse4"
)

# OS X specific options
if (APPLE)
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=gnu++11")
  set(CMAKE_CXX_LINK_FLAGS "-lc++")
else()
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
endif(APPLE)

# Add libraries
add_subdirectory(src)

# Main binary
add_executable(main "src/main.cpp")

# Depend on OpenCV and Boost
target_link_libraries(main pipeline ${OpenCV_LIBS} ${Boost_LIBRARIES})
