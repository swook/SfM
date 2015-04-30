cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLDataCapture)

set (CMAKE_BUILD_TYPE RelWithDebInfo)
# set (CMAKE_BUILD_TYPE Release)


### PCL dependent options

# If PCL can not be found, e.g. you installed it somewhere else than /usr/local/ under Linux specify below where the library is located
# set(PCL_DIR "/usr/local/share/pcl-1.5/")
# message (STATUS "PCL directory set to " ${PCL_DIR})

find_package(PCL 1.5 REQUIRED common io visualization)

# In case find_package can't find PCL you can also set the 3 following variables by hand.

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


### OpenCV dependent options

set (OpenCV_LIBS opencv_core opencv_highgui)

# option (1): use the package information from OpenCV to find the library
# recommended for Linux or if you compiled OpenCV yourself and installed it propperly
find_package (OpenCV REQUIRED core highgui)
if (OpenCV_FOUND)
	message (STATUS "Found OpenCV version " ${OpenCV_VERSION})
else()
	message (FATAL_ERROR : "OpenCV NOT FOUND!")
endif()

# option (2): define includes and libraries yourself
# use if (1) doesn't work for you, e.g. if you just downloaded the prebuild library (for Windows)
# Note: on Windows the libraries might include the version number, in this case use 'set (OpenCV_LIBS opencv_core231 opencv_highgui231)' above
# set (OpenCV_INCLUDE_DIRS "[...]/OpenCV/build/include")
# set (OpenCV_LIB_DIR "[...]/OpenCV/build/lib")
# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIB_DIR})


### Other library depdendencies

# If you want to use OpenMP for parallel processing uncomment the following
# This is not needed for the present data_capture executable
# find_package (OpenMP)
# if (OPENMP_FOUND)
# 	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# else()
#   message (FATAL_ERROR : "OpenMP NOT FOUND!")
# endif()


### Project dependent options
set (includes .)
include_directories (${includes})

add_executable (data_capture data_capture.cpp )
target_link_libraries (data_capture ${PCL_LIBRARIES} ${OpenCV_LIBS})

