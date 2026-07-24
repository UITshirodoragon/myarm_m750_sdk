cmake_minimum_required(VERSION 3.5)
project(myarm_m750_moveit_config)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
