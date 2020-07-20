cmake_minimum_required(VERSION 2.8.3)
project(moveit_floating_robot_planning)


set(DEPENDENCIES
  roscpp
  rviz
  octomap_server
  geometry_msgs
  visualization_msgs
  cmake_modules
  moveit_ros_planning_interface
  moveit_visual_tools
  topic_tools
)

find_package(catkin REQUIRED COMPONENTS ${DEPENDENCIES})

find_package(TinyXML REQUIRED)

### QT ###
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  find_package(Qt4 COMPONENTS REQUIRED QtCore QtGui QtOpenGL)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${QT_INCLUDE_DIR};${QT_QTCORE_INCLUDE_DIR};${QT_QTGUI_INCLUDE_DIR};${QT_QTOPENGL_INCLUDE_DIR}")
  set(Qt_LIBRARIES "${QT_QTCORE_LIBRARY};${QT_QTGUI_LIBRARY};${QT_QTOPENGL_LIBRARY}")
  set(Qt_LIBS
      Qt4::QtCore
      Qt4::QtGui
      Qt4::QtOpenGL
      )
else()
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(Qt5OpenGL REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
  set(Qt_LIBRARIES "${Qt5Core_LIBRARIES};${Qt5Gui_LIBRARIES};${Qt5OpenGL_LIBRARIES};${Qt5Widgets_LIBRARIES}")
  set(Qt_LIBS
      Qt5::Core
      Qt5::Gui
      Qt5::OpenGL
      Qt5::Widgets
      )
  add_definitions(-DWFlags=WindowFlags)
endif()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
    Qt
    TinyXML
  CATKIN_DEPENDS ${DEPENDENCIES} 
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Qt_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

set(UI_FILES
  ui/floating_robot_planning_frame.ui
)

set(SRC_FILES
  src/actions.cpp
  src/display_init.cpp
  src/octomaps.cpp
  src/XML.cpp
  src/plugin_init.cpp
)

set(HEADER_FILES
  include/${PROJECT_NAME}/actions.h
  include/${PROJECT_NAME}/display.h
)

if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  qt4_wrap_ui(UI_SRC_FILES ${UI_FILES})
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Qt_LIBRARIES}
    ${TinyXML_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS"
)

### Install the plugins ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)