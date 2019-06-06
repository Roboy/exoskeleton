cmake_minimum_required(VERSION 2.8.3)
project(exoskeleton)


find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
        gazebo_msgs
        roscpp
        std_msgs
        cmake_modules
        roboy_simulation_msgs
        )

find_package(gazebo REQUIRED)
find_package(OGRE)

#add_message_files(
#        FILES
#        MetabolicCost.msg
#)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES exo_motion_plugin metabolic_plugin joint_property_plugin
        CATKIN_DEPENDS roscpp std_msgs roboy_simulation_msgs
        #  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
list(APPEND CMAKE_MODULE_PATH cmake/FindOpenSim.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

# This depends on SimbodyConfig.cmake being located somewhere predictable
# on your machine. If you have installed it somewhere that CMake won't be
# able to guess, you'll need to tell find_package where to look.
find_package(Simbody REQUIRED)

include_directories(${Simbody_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${OGRE_LIBRARY_DIRS})
link_directories(${Simbody_LIB_DIR} ${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

find_package(OpenSim REQUIRED)
if (OPENSIM_FOUND)
    include_directories(${OPENSIM_INCLUDE_DIRS})
    link_directories(${OPENSIM_LIB_DIR})
    add_executable(metabolic_test src/metabolic_test.cpp)
    target_link_libraries(metabolic_test ${catkin_LIBRARIES}
            ${OPENSIMSIMBODY_LIBRARIES}
            ${OPENSIM_LIBRARIES}
            ${GAZEBO_LIBRARIES})
endif (OPENSIM_FOUND)

add_library(exo_motion_plugin src/exo_motion.cc)
target_link_libraries(exo_motion_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES})

install(TARGETS exo_motion_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

add_library(metabolic_plugin src/metabolic_plugin.cc)
add_dependencies(metabolic_plugin roboy_simulation_msgs_gencpp)
target_link_libraries(metabolic_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES})

install(TARGETS metabolic_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

add_library(joint_property_plugin src/joint_property_plugin.cc)
target_link_libraries(joint_property_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES})

install(TARGETS joint_property_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )