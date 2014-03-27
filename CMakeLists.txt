cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0005 OLD)
set(CMAKE_CXX_FLAGS "-std=c++0x") # -g for debugging 

PROJECT(simple_homing)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")


FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

ADD_LIBRARY(${PROJECT_NAME}_file_parser STATIC src/file_parser.cpp include/file_parser.h)
ADD_LIBRARY(${PROJECT_NAME}_yarp_interface STATIC src/yarp_interface.cpp)
ADD_LIBRARY(${PROJECT_NAME}_lib STATIC src/simple_homing.cpp)
ADD_EXECUTABLE(simple_homing src/main.cpp)

include_directories()
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_file_parser ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

SET(definitionsYarpInterface USE_POSITION_CONTROL_LEFT_ARM=true USE_POSITION_CONTROL_RIGHT_ARM=true)
SET_TARGET_PROPERTIES(${PROJECT_NAME}_yarp_interface PROPERTIES
                COMPILE_DEFINITIONS "${definitionsYarpInterface}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_yarp_interface ${YARP_LIBRARIES} icubmod ${drc_shared_LIBRARIES} ${ICUB_LIBRARIES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_lib 	${PROJECT_NAME}_yarp_interface 
						${PROJECT_NAME}_file_parser 
						${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

TARGET_LINK_LIBRARIES(simple_homing ${PROJECT_NAME}_lib 
									${PROJECT_NAME}_yarp_interface 
									${PROJECT_NAME}_file_parser 
									${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

add_custom_target(  copy_conf_files_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/app/conf"
        "${CMAKE_BINARY_DIR}/")

add_subdirectory(app)

install(TARGETS simple_homing DESTINATION bin)
