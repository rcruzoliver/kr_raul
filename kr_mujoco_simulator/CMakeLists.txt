cmake_minimum_required(VERSION 3.8)
project(kr_mujoco_simulation)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(COMMON_FLAGS "-O2 -std=c++17 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(thesis_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

pkg_search_module(GLFW REQUIRED glfw3)

add_executable(kr_mujoco_simulator src/kr_mujoco_simulator.cpp)

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)

# Include directories
set(THESIS_INTERFACES_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/../../install/thesis_interfaces/include/thesis_interfaces")

target_include_directories(kr_mujoco_simulator
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${GLFW_INCLUDE_DIRS}
    ${THESIS_INTERFACES_INCLUDE}
)

target_include_directories(kr_mujoco_simulator
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(mujoco_gym -Wl,--no-undefined)
target_link_libraries(sim_time -Wl,--no-undefined)

target_link_libraries(mujoco_gym
  mujoco
  ${GLFW_LIBRARIES}
  Eigen3::Eigen
)


# Link against necessary libraries
ament_target_dependencies(kr_mujoco_simulator
  rclcpp
  std_msgs
  sensor_msgs
  rosgraph_msgs
  thesis_interfaces
  Eigen3
)



set(MUJOCO_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/")

set_target_properties(mujoco_gym PROPERTIES
    INSTALL_RPATH ${MUJOCO_PATH}
    INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS mujoco_gym
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY robot 
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
