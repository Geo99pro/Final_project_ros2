cmake_minimum_required(VERSION 3.5)
project(gpg_remote)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gpg_remote_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gpg_remote_hw.cpp
                                   src/gpg_remote_broadcaster.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface controller_interface pluginlib gpg_remote_msgs)
pluginlib_export_plugin_description_file(hardware_interface gpg_remote_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface gpg_remote_controller.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
        DESTINATION include)
install(PROGRAMS scripts/fake_data_server
                 scripts/get_battery
                 scripts/image_publisher
                 scripts/model_state_to_pose
                 scripts/setvel
                 scripts/teleop
        DESTINATION lib/${PROJECT_NAME})
install(FILES launch/gpg.launch.py
              config/camera.yaml
              config/controllers.yaml
              config/gopigo3.urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()

