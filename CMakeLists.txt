cmake_minimum_required(VERSION 3.8)
project(stereo_aruco_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc imgcodecs aruco)

add_library(stereo_calibration_core
  src/board_model.cpp src/sample_manager.cpp src/stability_gate.cpp src/stereo_solver.cpp src/result_writer.cpp)
set_target_properties(stereo_calibration_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stereo_calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_calibration_core PUBLIC ${OpenCV_LIBS})

add_library(stereo_calibration_component SHARED src/calibrator_node.cpp)
target_include_directories(stereo_calibration_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(stereo_calibration_component stereo_calibration_core)
ament_target_dependencies(stereo_calibration_component
  rclcpp rclcpp_components sensor_msgs std_srvs diagnostic_msgs cv_bridge)
rclcpp_components_register_node(stereo_calibration_component
  PLUGIN "stereo_calibration::StereoArucoCalibratorNode"
  EXECUTABLE stereo_aruco_calibrator_node)

install(TARGETS stereo_calibration_core stereo_calibration_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/stereo_calibration_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stereo_calibration_gui)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_board_model test/test_board_model.cpp)
  target_link_libraries(test_board_model stereo_calibration_core)
  ament_add_gtest(test_stereo_solver test/test_stereo_solver.cpp)
  target_link_libraries(test_stereo_solver stereo_calibration_core)
  ament_add_gtest(test_stability_gate test/test_stability_gate.cpp)
  target_link_libraries(test_stability_gate stereo_calibration_core)
endif()
ament_package()
