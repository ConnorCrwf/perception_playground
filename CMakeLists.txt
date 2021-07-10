cmake_minimum_required(VERSION 3.0.2)
project(perception_playground)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(glfw3)
find_package(OpenGL)
find_package(GLUT)
find_package(realsense2)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_playground
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/perception_playground.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

add_executable(example_voxelgrid src/pcl/example_voxelgrid.cpp)
add_executable(example_planarsegmentation src/pcl/example_planarsegmentation.cpp)
add_executable(pcl_processing src/pcl/pcl_processing.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(example_voxelgrid ${catkin_LIBRARIES})
target_link_libraries(example_planarsegmentation ${catkin_LIBRARIES})
target_link_libraries(pcl_processing ${catkin_LIBRARIES})


add_executable(raw_record src/record/raw_record.cpp)
set_property(TARGET raw_record PROPERTY CXX_STANDARD 11)
target_link_libraries(raw_record realsense2 ${DEPENDENCIES})
include_directories(include/perception_playground/tclap/include)
# set_target_properties (raw_record PROPERTIES
#     FOLDER "Tools"
# )

add_executable(raw_record_v2 src/record/raw_record_v2.cpp)
set_property(TARGET raw_record_v2 PROPERTY CXX_STANDARD 11)
target_link_libraries(raw_record_v2 realsense2 ${DEPENDENCIES} ${catkin_LIBRARIES})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(my_rs-align src/record/my_rs-align.cpp include/perception_playground/imgui/imgui.cpp include/perception_playground/imgui/imgui_draw.cpp include/perception_playground/imgui/imgui_impl_glfw.cpp)
set_property(TARGET my_rs-align PROPERTY CXX_STANDARD 11)
target_link_libraries(my_rs-align PRIVATE realsense2::realsense2 glfw ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${DEPENDENCIES})
include_directories(include/perception_playground/imgui examples ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
# set(${GLUT_LIBRARY} -lGLU)
# or PRIVATE ${OPENGL_gl_LIBRARY}
# PRIVATE OpenGL::OpenGL ${GL_LIBRARY}
# ${GLFW_STATIC_LIBRARIES}
# ${GLFW_INCLUDE_DIRS}
# include/perception_playground/glfw/include include/perception_playground/glad
# set_target_properties (my_rs-align PROPERTIES FOLDER Examples)

add_executable(my_rs-record-playback src/record/my_rs-record-playback.cpp include/perception_playground/imgui/imgui.cpp include/perception_playground/imgui/imgui_draw.cpp include/perception_playground/imgui/imgui_impl_glfw.cpp)
set_property(TARGET my_rs-record-playback PROPERTY CXX_STANDARD 11)
target_link_libraries(my_rs-record-playback PRIVATE realsense2::realsense2 glfw ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${DEPENDENCIES})

