cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS roscpp arc_tools geometry_msgs nav_msgs ackermann_msgs catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
	src/pure_pursuit_controller.cpp
)

#include_directories(include ${catkin_INCLUDE_DIRS})






## Declare a C++ executable
add_executable(regler_node src/pure_pursuit_controller_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
##add_dependencies(pure_pursuit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(regler_node
   ${catkin_LIBRARIES}
${PROJECT_NAME}
 )

cs_install()
cs_export()
