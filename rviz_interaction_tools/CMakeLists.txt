cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
)

set(SOURCE_FILES
  src/mesh_object.cpp 
  src/mesh_object_switcher.cpp
  src/image_overlay.cpp
  src/disparity_renderer.cpp
  src/camera_tools.cpp
  src/point_cloud.cpp
  src/gripper.cpp
  src/cartesian_control.cpp
  src/line.cpp
  ${MOC_FILES}
)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
                                    
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})

#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories( ${wxWidgets_INCLUDE_DIRS} )

#pkg_check_modules(OGRE OGRE)
#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )
                                    
#target_link_libraries(rviz_interaction_ui ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} gripper_click_ui)
