cmake_minimum_required(VERSION 2.8.3)
project(octomap_rrt_planner)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
find_package(ompl REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_rrt_planner.cpp
  src/octomap_ompl_rrt.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})


############
# BINARIES #
############
cs_add_executable(octomap_rrt_planner_node
  src/octomap_rrt_planner_node.cpp
)
target_link_libraries(octomap_rrt_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})