cmake_minimum_required(VERSION 2.8.3)
project(hero_agent_v2)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

add_message_files(FILES MsgTutorial.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hero_agent_v2
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(agent_main src/agent_main.cpp)
add_dependencies(agent_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(agent_main ${catkin_LIBRARIES})

add_executable(agent_dvl src/agent_dvl.cpp)
add_dependencies(agent_dvl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(agent_dvl ${catkin_LIBRARIES})