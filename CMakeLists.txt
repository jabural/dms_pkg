cmake_minimum_required(VERSION 3.8)
project(dms_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)


# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
  DEPENDENCIES
  action_msgs
  ADD_LINTER_TESTS
)

# Set support for using custom interfaces in C++ code
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  rclcpp_action
  action_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(main src/main.cpp)
ament_target_dependencies(main ${dependencies})
target_link_libraries(main "${cpp_typesupport_target}")

# add_executable(talker src/talker_node.cpp)
# ament_target_dependencies(talker ${dependencies})
# target_link_libraries(talker "${cpp_typesupport_target}")

# add_executable(listener src/listener_node.cpp)
# ament_target_dependencies(listener ${dependencies})
# target_link_libraries(listener "${cpp_typesupport_target}")

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server ${dependencies})
target_link_libraries(action_server "${cpp_typesupport_target}")


install(TARGETS
  main
  action_server
  # talker
  # listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_pytest_test(test_action test/test_action.py
   TIMEOUT 60
  )

  # ament_add_gtest(minimal_test
  #   test/tutorial_test.cpp
  # )

  # ament_target_dependencies(minimal_test
  #   ${dependencies}
  # )

  # target_link_libraries(minimal_test "${cpp_typesupport_target}")
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_uncrustify_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
