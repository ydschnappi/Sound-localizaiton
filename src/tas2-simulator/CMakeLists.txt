# ===================================================================================
# Description: CMakelists file for TAS2 simulator.
# Author: Salman Bari
# Date: Sep 5, 2022
# ===================================================================================

cmake_minimum_required(VERSION 3.8)
project(tas2-simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#install
install(
  DIRECTORY config launch maps models rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
