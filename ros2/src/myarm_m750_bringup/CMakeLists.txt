cmake_minimum_required(VERSION 3.5)
project(myarm_m750_bringup)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    test_network_environment
    COMMAND
      "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_network_environment.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
