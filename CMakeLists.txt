cmake_minimum_required(VERSION 3.2.1)
project(autonomous_wheelchair)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
  nav_core 
  move_base_msgs
  base_local_planner
  roscpp
  std_msgs
  tf
  rostest
  roscpp
  rospy
  message_generation)

catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS nav_core roscpp std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


###########
## Test ##
###########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  ##find_package(gtest)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  catkin_add_gtest(testGrid test/testGridSquare.cpp src/gridSquare.cpp)
  target_link_libraries(testGrid ${catkin_LIBRARIES})
  add_dependencies(testGrid ${catkin_EXPORTED_TARGETS})

  add_rostest_gtest(testPlanner launch/test/testPathPlanner.launch test/main.cpp test/testPathPlanner.cpp src/gridSquare.cpp src/pathPlanner.cpp)
  target_link_libraries(testPlanner ${catkin_LIBRARIES})
  add_dependencies(testPlanner pathPlanner ${catkin_EXPORTED_TARGETS})

  add_rostest_gtest(testNavigator launch/test/testNavigator.launch test/testNavigateRobot.cpp src/navigateRobot.cpp)
  target_link_libraries(testNavigator ${catkin_LIBRARIES})
  add_dependencies(testNavigator navigate ${catkin_EXPORTED_TARGETS})
endif()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pathPlanner src/gridSquare.cpp src/pathPlanner.cpp)
target_link_libraries(pathPlanner ${catkin_LIBRARIES})

add_executable(goal src/goal.cpp)
target_link_libraries(goal ${catkin_LIBRARIES})

add_executable(navigate src/main.cpp src/navigateRobot.cpp)
target_link_libraries(navigate ${catkin_LIBRARIES})
