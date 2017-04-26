# Copyright: (C) 2015 iCub Facility
# Authors: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(safeMotionControl CATEGORY device TYPE SafeMotionControl INCLUDE safeMotionControl.h)

if(NOT SKIP_safeMotionControl)

get_property(R1ModelLib_INCLUDE_DIRS
             TARGET R1ModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(RobotModelLib_INCLUDE_DIRS
             TARGET RobotModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${R1ModelLib_INCLUDE_DIRS}
                    ${RobotModelLib_INCLUDE_DIRS})

yarp_add_plugin(safeMotionControl safeMotionControl.h safeMotionControl.cpp)
target_link_libraries(safeMotionControl ${YARP_LIBRARIES} R1ModelLib RobotModelLib)

icub_export_plugin(safeMotionControl)

yarp_install(FILES safeMotionControl.ini DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

endif (NOT SKIP_safeMotionControl)


