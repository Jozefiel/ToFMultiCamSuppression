cmake_minimum_required(VERSION 3.5)

#set project name
set(PROJECT "Interference")
project(${PROJECT})

#speed up pcl compiling
find_program(CCACHE_FOUND ccache REQUIRED)
if(CCACHE_FOUND)
 set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
 set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)
set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")

#find requirements
find_package(PCL 1.9.1 REQUIRED )
find_package(OpenCV REQUIRED )
find_package(VTK REQUIRED )
find_package(Threads )

#link libs
include_directories("/usr/include/libusb-1.0/")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#load src directory
file(GLOB_RECURSE SRCS *.cpp)
add_executable(${PROJECT}  ${SRCS})

target_link_libraries(${PROJECT} ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )


#compilator settings

ADD_DEFINITIONS(
    -std=c++14 # Or -std=c++0x
    # O
)

set_target_properties(${PROJECT} PROPERTIES
        CXX_STANDARD_REQUIRED 14
        CXX_EXTENSIONS ON
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")

