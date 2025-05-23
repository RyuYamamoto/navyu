cmake_minimum_required(VERSION 3.5)
project(navyu_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(navyu_utils SHARED
  src/navyu_utils.cpp
)

ament_auto_add_library(quaternion_utils SHARED
  src/quaternion_utils.cpp
)

ament_auto_add_library(costmap_helper SHARED
  src/costmap_helper.cpp
)

ament_auto_add_library(visualization_utils SHARED
  src/visualization_utils.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
