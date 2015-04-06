cmake_minimum_required(VERSION 2.8)

project(sfm)

# Add libraries
find_package(OpenCV REQUIRED core highgui imgproc)
find_package(Boost  REQUIRED program_options system filesystem date_time)

include_directories(${Boost_INCLUDE_DIRS})
#set(Boost_USE_STATIC_LIBS ON)

# Set compile options
set(
  CMAKE_CXX_FLAGS
  ${CMAKE_CXX_FLAGS}
  "-O3 -std=c++11 -msse3 -msse4"
)

# Add libraries
add_subdirectory(src)

# Main binary
add_executable(main "src/main.cpp")

# Depend on OpenCV and Boost
target_link_libraries(main pipeline ${OpenCV_LIBS} ${Boost_LIBRARIES})
