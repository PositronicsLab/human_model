cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(Human)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

Include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif(PKG_CONFIG_FOUND)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# look for Pacer
find_library(PACER_FOUND Pacer REQUIRED /usr/local/lib /usr/lib)
add_definitions (-DUSE_INV_DYN)

find_library(SDFORMAT_FOUND SDFormat REQUIRED /usr/local/lib /usr/lib)

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

find_package(xacro REQUIRED)

add_executable(human-pacer src/human-pacer.cpp)
add_executable(combined-human-pr2-pacer src/combined-human-pr2-pacer.cpp)
add_library(robot-controller src/robot-controller.cpp)
target_link_libraries(human-pacer Moby Pacer Ravelin)
target_link_libraries(combined-human-pr2-pacer Pacer Ravelin)
target_link_libraries(robot-controller Ravelin Opt)
install(TARGETS human-pacer DESTINATION ${CMAKE_SOURCE_DIR}/human-pacer)

add_library(ContactForcesPlugin src/contact-forces-plugin.cpp)
target_link_libraries(ContactForcesPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

# Gazebo controller plugin
add_library(StableControllerPlugin SHARED src/stable-controller-plugin.cpp)
target_link_libraries(StableControllerPlugin Moby Ravelin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf SDFormat)
install(TARGETS StableControllerPlugin DESTINATION lib)

add_library(InitialJointVelocityPlugin SHARED src/initial-joint-velocity-plugin.cpp)
target_link_libraries(InitialJointVelocityPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS InitialJointVelocityPlugin DESTINATION lib)

set(model_files "")

# Xacro files
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/models/*/*.xacro)

foreach(it ${model_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  
  message(" processing world file :",${basepath}," : ",${basename})
  
  set(expanded_file "${basepath}/${basename}.generated.sdf")
  message("    produce expanded file :",${expanded_file})
  
  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(model_files ${model_files} ${expanded_file})
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${model_files})
