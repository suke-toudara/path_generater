cmake_minimum_required(VERSION 3.8)
project(path_generater)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()


find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_auto_find_build_dependencies()

ament_auto_add_library(path_generater SHARED
 src/path_generater.cpp)

rclcpp_components_register_nodes(path_generater
  "path_generater::PathGenerater"
)

ament_auto_add_executable(path_generater_node
    NO_TARGET_LINK_LIBRARIES
    src/path_generater_node.cpp
)

target_link_libraries(path_generater_node
  path_generater
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package()


