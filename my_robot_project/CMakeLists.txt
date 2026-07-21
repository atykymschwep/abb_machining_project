cmake_minimum_required(VERSION 3.8)
project(my_robot_project)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(abb_irb4600_ikfast REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# model_spawner
add_executable(model_spawner src/model_spawner.cpp)
target_include_directories(model_spawner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(model_spawner rclcpp visualization_msgs)

# pose_teacher
add_executable(pose_teacher src/pose_teacher.cpp)
target_include_directories(pose_teacher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(pose_teacher 
    rclcpp 
    visualization_msgs 
    interactive_markers 
    sensor_msgs 
    geometry_msgs 
    abb_irb4600_ikfast
)

# trajectory_planner
add_executable(trajectory_planner src/trajectory_planner.cpp)
target_include_directories(trajectory_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(trajectory_planner 
    rclcpp 
    sensor_msgs 
    geometry_msgs
    abb_irb4600_ikfast
    std_srvs 
    visualization_msgs
)

# install
install(TARGETS
  model_spawner
  pose_teacher
  trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()