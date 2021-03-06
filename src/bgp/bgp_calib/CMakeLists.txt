cmake_minimum_required(VERSION 2.8.3)
project(bgp_calib)

add_definitions(-std=c++11 -g)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  geometry_msgs
  apriltag_msgs
)

find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 3 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS})

#target_link_libraries(calib_intrinsics ${CERES_LIBRARIES}
#${OpenCV_LIBRARIES})


# what to export
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs roscpp
	)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/CalibNode.cpp src/CalibTool.cpp src/Camera.cpp src/utils.cpp src/Tag.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${GTSAM_LIBRARIES}
  ${catkin_LIBRARIES}
)

