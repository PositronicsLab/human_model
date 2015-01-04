cmake_minimum_required(VERSION 2.8.3)

project(Human)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif(PKG_CONFIG_FOUND)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# look for Pacer
find_library(PACER_FOUND Pacer REQUIRED /usr/local/lib /usr/lib)
add_definitions (-DUSE_INV_DYN)
# fix the C++ linking error on 64-bit Linux
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)
add_definitions(-DTIMING)

include_directories(
                    include
                    /usr/include
                    /usr/include/libxml2
                    /usr/local/include
                    ${Boost_INCLUDE_DIR}
                    /opt/local/include
                    /opt/X11/include
                    )

link_directories(
                  /usr/local/lib
                  /opt/X11/lib
                  )

set(VISUALIZE_LIBS glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
FIND_PACKAGE( GLUT REQUIRED )
FIND_PACKAGE( OpenGL REQUIRED )

add_executable(human-pacer human_pacer.cpp)
add_executable(combined-human-pr2-pacer combined_human_pr2_pacer.cpp)
add_library(robot-controller robot-controller.cpp)
target_link_libraries(human-pacer Pacer Ravelin)
target_link_libraries(combined-human-pr2-pacer Pacer Ravelin)
target_link_libraries(robot-controller Ravelin Opt)
install(TARGETS human-pacer DESTINATION ${CMAKE_SOURCE_DIR}/human-pacer)

# Gazebo controller plugin
add_library(StableControllerPlugin SHARED stable-controller-plugin.cpp)
target_link_libraries(StableControllerPlugin Moby Ravelin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS StableControllerPlugin DESTINATION lib)
