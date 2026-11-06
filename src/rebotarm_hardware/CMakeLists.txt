cmake_minimum_required(VERSION 3.8)
project(rebotarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Reuse the MotorBridge ABI shipped with the installed Python SDK.
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import motorbridge, pathlib; print(pathlib.Path(motorbridge.__file__).parent / 'lib' / 'libmotor_abi.so')"
  OUTPUT_VARIABLE MOTORBRIDGE_PY_ABI_LIBRARY
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
set(MOTORBRIDGE_ABI_LIBRARY "${MOTORBRIDGE_PY_ABI_LIBRARY}" CACHE FILEPATH
  "Path to the MotorBridge v0.5.6 libmotor_abi.so")
if(NOT EXISTS "${MOTORBRIDGE_ABI_LIBRARY}")
  message(FATAL_ERROR "MotorBridge ABI library not found. Set MOTORBRIDGE_ABI_LIBRARY.")
endif()

add_library(rebotarm_hardware_interface SHARED
  src/rebotarm_sdk.cpp
  src/rebotarm_hardware_interface.cpp)
target_compile_features(rebotarm_hardware_interface PUBLIC cxx_std_17)
target_include_directories(rebotarm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rebotarm_hardware_interface
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(rebotarm_hardware_interface "${MOTORBRIDGE_ABI_LIBRARY}")
set_target_properties(rebotarm_hardware_interface PROPERTIES INSTALL_RPATH "$ORIGIN")

pluginlib_export_plugin_description_file(hardware_interface rebotarm_hardware.xml)
install(TARGETS rebotarm_hardware_interface DESTINATION lib)
install(FILES "${MOTORBRIDGE_ABI_LIBRARY}" DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES MOTORBRIDGE_LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
