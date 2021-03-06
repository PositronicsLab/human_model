cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(Human)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

Include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
                    /usr/local/include/sdformat-2.3
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

add_library(ContactForcesPlugin SHARED src/contact-forces-plugin.cpp)
target_link_libraries(ContactForcesPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
install(TARGETS ContactForcesPlugin DESTINATION lib)

add_library(ContactForcesPerLinkOverTimePlugin SHARED src/contact-forces-per-link-over-time-plugin.cpp)
target_link_libraries(ContactForcesPerLinkOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
install(TARGETS ContactForcesPerLinkOverTimePlugin DESTINATION lib)

# Gazebo controller plugin
add_library(StableControllerPlugin SHARED src/stable-controller-plugin.cpp)
target_link_libraries(StableControllerPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf SDFormat)
install(TARGETS StableControllerPlugin DESTINATION lib)

add_library(InitialJointVelocityPlugin SHARED src/initial-joint-velocity-plugin.cpp)
target_link_libraries(InitialJointVelocityPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS InitialJointVelocityPlugin DESTINATION lib)

add_library(InitialJointPositionPlugin SHARED src/initial-joint-position-plugin.cpp src/chainiksolverpos_nr_jl_position_only.cpp)
target_link_libraries(InitialJointPositionPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} protobuf fcl SDFormat)
install(TARGETS InitialJointPositionPlugin DESTINATION lib)

add_library(JointAnglesPlugin SHARED src/joint-angles-plugin.cpp)
target_link_libraries(JointAnglesPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS JointAnglesPlugin DESTINATION lib)

add_library(JointForcesPlugin SHARED src/joint-forces-plugin.cpp)
target_link_libraries(JointForcesPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS JointForcesPlugin DESTINATION lib)

add_library(MinMaxJointPositionPlugin SHARED src/min-max-joint-position-plugin.cpp)
target_link_libraries(MinMaxJointPositionPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS MinMaxJointPositionPlugin DESTINATION lib)

add_library(ArmCalibrationPlugin SHARED src/arm-calibration-plugin.cpp)
target_link_libraries(ArmCalibrationPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS ArmCalibrationPlugin DESTINATION lib)

add_library(ContactForcesOverTimePlugin SHARED src/contact-forces-over-time-plugin.cpp)
target_link_libraries(ContactForcesOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS ContactForcesOverTimePlugin DESTINATION lib)

add_library(VelocityOverTimePlugin SHARED src/com-velocity-over-time-plugin.cpp)
target_link_libraries(VelocityOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS VelocityOverTimePlugin DESTINATION lib)

add_library(COMAccOverTimePlugin SHARED src/com-acc-over-time-plugin.cpp)
target_link_libraries(COMAccOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS COMAccOverTimePlugin DESTINATION lib)

add_library(LinkAccOverTimePlugin SHARED src/link-acc-over-time-plugin.cpp)
target_link_libraries(LinkAccOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS LinkAccOverTimePlugin DESTINATION lib)

add_library(InitialLinkPositionPlugin SHARED src/initial-link-position-plugin.cpp)
target_link_libraries(InitialLinkPositionPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS InitialLinkPositionPlugin DESTINATION lib)

add_library(AngularMomentumOverTimePlugin SHARED src/angular-momentum-over-time-plugin.cpp)
target_link_libraries(AngularMomentumOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS AngularMomentumOverTimePlugin DESTINATION lib)

add_library(InitialLinkEnergyPlugin SHARED src/initial-link-energy-plugin.cpp)
target_link_libraries(InitialLinkEnergyPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS InitialLinkEnergyPlugin DESTINATION lib)

add_library(LinkEnergyOverTimePlugin SHARED src/link-energy-over-time-plugin.cpp)
target_link_libraries(LinkEnergyOverTimePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS LinkEnergyOverTimePlugin DESTINATION lib)

add_library(SetHumanPositionPlugin SHARED src/set-human-position-plugin.cpp)
target_link_libraries(SetHumanPositionPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
install(TARGETS SetHumanPositionPlugin DESTINATION lib)

set(model_files "")

# Xacro files
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/models/*/*.xacro)

foreach(it ${model_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  
  message(" processing world file :",${basepath}," : ", ${basename})
  
  set(expanded_file "${basepath}/${basename}.generated.sdf")
  message("    produce expanded file :", ${expanded_file})
  
  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(model_files ${model_files} ${expanded_file})
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${model_files})
