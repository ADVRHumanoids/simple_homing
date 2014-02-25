cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0005 OLD)

PROJECT(simple_homing)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")


FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

ADD_LIBRARY(file_parser src/file_parser.cpp include/file_parser.h)
ADD_LIBRARY(yarp_interface src/yarp_interface.cpp)
ADD_LIBRARY(simple_homing_lib src/simple_homing.cpp)
ADD_EXECUTABLE(simple_homing src/main.cpp)

TARGET_LINK_LIBRARIES(file_parser ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

SET(definitionsYarpInterface USE_POSITION_CONTROL_LEFT_ARM=true USE_POSITION_CONTROL_RIGHT_ARM=true)
SET_TARGET_PROPERTIES(yarp_interface PROPERTIES
                COMPILE_DEFINITIONS "${definitionsYarpInterface}")
TARGET_LINK_LIBRARIES(yarp_interface ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

TARGET_LINK_LIBRARIES(simple_homing_lib yarp_interface file_parser ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

TARGET_LINK_LIBRARIES(simple_homing simple_homing_lib yarp_interface file_parser ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

add_custom_command(TARGET simple_homing POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/app/conf"
        "${CMAKE_BINARY_DIR}/")

add_subdirectory(app)

install(TARGETS simple_homing DESTINATION bin)
