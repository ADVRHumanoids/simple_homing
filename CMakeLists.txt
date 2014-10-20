cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0005 OLD)

project(simple_homing)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${drc_shared_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS} )
LINK_DIRECTORIES(${ROS_LIB_FOLDER} ${WALKMAN_LIB_FOLDER})

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

ADD_LIBRARY(${PROJECT_NAME}_lib STATIC src/simple_homing.cpp)
ADD_EXECUTABLE(simple_homing src/main.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_lib 	${YARP_LIBRARIES}
						${drc_shared_LIBRARIES}
						${paramHelp_LIBRARIES})

TARGET_LINK_LIBRARIES(simple_homing ${PROJECT_NAME}_lib 
                                    ${YARP_LIBRARIES}
                                    ${drc_shared_LIBRARIES}
                                    ${paramHelp_LIBRARIES}
)

add_custom_target(  copy_conf_files_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/app/conf"
        "${CMAKE_BINARY_DIR}/")

add_subdirectory(app)

install(TARGETS simple_homing DESTINATION bin)
