cmake_minimum_required(VERSION 2.8.12)
project(decision_maker)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_config_msgs
  autoware_msgs
  geometry_msgs
  jsk_recognition_msgs
  jsk_rviz_plugins
  lanelet2_extension
  roscpp
  roslint
  state_machine_lib
  std_msgs
  tf2
  tf2_ros
  vector_map
  vector_map_msgs
  visualization_msgs
)

catkin_package()

set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/cross_road_area.cpp
  src/decision_maker_node_callback.cpp
  src/decision_maker_node_core.cpp
  src/decision_maker_node_decision.cpp
  src/decision_maker_node_init.cpp
  src/decision_maker_node_publish.cpp
  src/decision_maker_node_state_behavior.cpp
  src/decision_maker_node_state_mission.cpp
  src/decision_maker_node_state_motion.cpp
  src/decision_maker_node_state_vehicle.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_node
  src/decision_maker_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  FILES
  autoware_state_behavior.yaml
  autoware_state_motion.yaml
  autoware_state_mission.yaml
  autoware_state_vehicle.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()

  find_package(rostest REQUIRED)
  add_rostest_gtest(
    test-decision_maker
    test/test_decision_maker.test
    test/src/test_main.cpp
    test/src/test_node_decision.cpp
    test/src/test_node_lanelet2_functions.cpp
    test/src/test_node_state_drive.cpp
  )
  add_dependencies(test-${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(
    test-${PROJECT_NAME}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()
