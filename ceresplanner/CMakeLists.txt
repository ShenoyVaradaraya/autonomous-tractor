cmake_minimum_required(VERSION 3.0.2)
project(ceresplanner)
set(CMAKE_CXX_STANDARD 17)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(OpenCV REQUIRED)
# find_package(Eigen3 3.3 REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)
# find_package (GTest REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
#   flux_msgs
  roslib
#   costmap_msgs
#   dynamic_reconfigure
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp roslib
)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# # enable_testing()
add_executable(PP_ceres src/ceres_main.cpp src/ceresplanner.cpp src/line.cpp src/subregion.cpp src/transform.cpp src/bezier.cpp)

target_link_libraries(PP_ceres
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# target_link_libraries(
# gtest_planner
# GTest::gtest_main
# ${catkin_LIBRARIES}
# ${OpenCV_LIBS}
# )

# include(GoogleTest)
# gtest_discover_tests(gtest_planner)
# add_dependencies(
#   PP_Junctions flux_msgs_generate_messages_cpp costmap_msgs_generate_messages_cpp
# )
# add_dependencies(
#   PP_GlobalPlanner flux_msgs_generate_messages_cpp costmap_msgs_generate_messages_cpp
# )
# add_dependencies(
#   PP_Recovery flux_msgs_generate_messages_cpp costmap_msgs_generate_messages_cpp
# )
# add_dependencies(
#   PP_CorrectOrien flux_msgs_generate_messages_cpp costmap_msgs_generate_messages_cpp
# )
# add_dependencies(
#   PP_DwaPlanner flux_msgs_generate_messages_cpp costmap_msgs_generate_messages_cpp
# )

# add_dependencies(PP_Config ${PROJECT_NAME}_gencfg)

# install(DIRECTORY cfg
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# catkin_install_python(PROGRAMS
#   scripts/junction_creator.py
#   scripts/tracker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# #See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS PP_GlobalPlanner
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS PP_Recovery
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS PP_CorrectOrien
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS PP_DwaPlanner
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS PP_Junctions
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS PP_Config
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# if (CATKIN_ENABLE_TESTING)
# catkin_add_gtest(gtest_planner 
#   gtest_planner/gtest_planner.cpp
#  gtest_planner/gtest_bismooth.cpp 
#  src/planner.cpp src/bezier.cpp src/bi_smooth.cpp src/image_process.cpp src/transform.cpp src/vehicle_fp.cpp src/utility_func.cpp)
# if(TARGET gtest_planner)
# target_link_libraries(gtest_planner GTest::gtest_main ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
# endif()
# endif()