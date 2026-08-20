cmake_minimum_required(VERSION 3.8)
project(surface_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_collision_checker REQUIRED)
find_package(ur_ik_solver REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdfdom REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_core REQUIRED)
find_package(graph_display REQUIRED)
find_package(ik_solver REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(ament_cmake_python REQUIRED)

find_package(control_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



set(DEPS_LIB
    graph_core
    )
set(DEPS_TARGET
    graph_core::graph_core
    )

set(ROS_DEPS
    rclcpp
    graph_display
    visualization_msgs
    moveit_visual_tools
    tf2_eigen
    ik_solver
    moveit_collision_checker
    ik_solver_core
    moveit_core
    moveit_ros_planning_interface
    )



add_library(surface_path_planning SHARED
    src/surface_path_planning/utils.cpp
    src/surface_path_planning/surface_path_planning.cpp
    src/surface_path_planning/robot_description_node.cpp
)
target_include_directories(surface_path_planning PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(surface_path_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(surface_path_planning
  PUBLIC
    graph_core::graph_core
    ik_solver::ik_solver
    graph_display::graph_display
    moveit_collision_checker::moveit_collision_checker
)
# Create an alias for the library to link against easily
add_library(surface_path_planning::surface_path_planning ALIAS surface_path_planning)

# Simple test 1
add_executable(simple_test_1 src/simple_test_1.cpp)
target_include_directories(simple_test_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(simple_test_1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(simple_test_1
    surface_path_planning::surface_path_planning
)
ament_target_dependencies(simple_test_1 control_msgs ${ROS_DEPS})


# Simple test 2
add_executable(simple_test_2 src/simple_test_2.cpp)
target_include_directories(simple_test_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(simple_test_2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(simple_test_2
    surface_path_planning::surface_path_planning
)
ament_target_dependencies(simple_test_2 control_msgs ${ROS_DEPS})


#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})

install(TARGETS
        simple_test_1 simple_test_2 surface_path_planning
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})
    
ament_package()
