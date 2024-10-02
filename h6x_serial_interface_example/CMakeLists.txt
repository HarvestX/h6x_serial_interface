# Copyright (c) 2024 HarvestX Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(h6x_serial_interface_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Simple Read Node ==================================================
set(TARGET simple_read_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::SimpleReadNode"
  EXECUTABLE ${TARGET}_exec)

# Simple ReadUntil Node ==================================================
set(TARGET simple_read_until_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::SimpleReadUntilNode"
  EXECUTABLE ${TARGET}_exec)

# Simle Write Node ==================================================
set(TARGET simple_write_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::SimpleWriteNode"
  EXECUTABLE ${TARGET}_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
