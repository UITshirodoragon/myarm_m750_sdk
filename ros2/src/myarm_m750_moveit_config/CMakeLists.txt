cmake_minimum_required(VERSION 3.5)
project(myarm_m750_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    test_model_catalog
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_model_catalog.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(
    test_moveit_config_contract
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_moveit_config_contract.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
