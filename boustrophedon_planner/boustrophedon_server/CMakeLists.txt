cmake_minimum_required(VERSION 2.8.3)
project(boustrophedon_server)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  boustrophedon_msgs
  actionlib
  tf
  geometry_msgs
  nav_msgs
  rosparam_shortcuts
  )

find_package(CGAL REQUIRED COMPONENTS Core)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  boustrophedon_msgs
  actionlib
  tf
  geometry_msgs
  nav_msgs
  rosparam_shortcuts
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(boustrophedon_planner_server
  src/boustrophedon_server/boustrophedon_planner_node.cpp
  src/boustrophedon_server/boustrophedon_planner_server.cpp
  src/boustrophedon_server/striping_planner.cpp
  src/boustrophedon_server/outline_planner.cpp
  src/boustrophedon_server/cgal_utils.cpp
  src/boustrophedon_server/boustrophedon_types.cpp
  src/boustrophedon_server/cellular_decomposition/polygon_decomposer.cpp
  src/boustrophedon_server/cellular_decomposition/cell.cpp
  src/boustrophedon_server/bezier.cpp
  )

add_executable(boustrophedon_client src/boustrophedon_server/boustrophedon_planner_client.cpp)
add_executable(local_planner src/boustrophedon_server/local_planner_main.cpp src/boustrophedon_server/local_planner.cpp src/boustrophedon_server/bezier.cpp src/boustrophedon_server/vel_prof.cpp)


target_link_libraries(boustrophedon_planner_server
  ${catkin_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
  ${OpenCV_LIBS})

target_link_libraries(boustrophedon_client
  ${catkin_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
  ${OpenCV_LIBS})
target_link_libraries(local_planner
  ${catkin_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
  ${OpenCV_LIBS})

add_dependencies(boustrophedon_planner_server ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
  boustrophedon_planner_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
