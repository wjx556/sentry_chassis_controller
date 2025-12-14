cmake_minimum_required(VERSION 3.0.2)
project(sentry_chassis_controller)

# 1. 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  control_toolbox
  realtime_tools
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure # <--- 必须有这个
)

# 2. 生成动态参数配置文件 (必须在 catkin_package 之前)
generate_dynamic_reconfigure_options(
  cfg/SentryPids.cfg
)

# 3. 声明 Catkin 包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sentry_chassis_controller
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib control_toolbox dynamic_reconfigure
)

# 4. 指定头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 5. 生成库文件 (重点检查这里！)
# 错误原因通常在这里：必须包含 .cpp 源文件，不能只有依赖名
add_library(sentry_chassis_controller
  src/sentry_chassis_controller.cpp
)

# 6. 添加库文件的链接库
target_link_libraries(sentry_chassis_controller
  ${catkin_LIBRARIES}
)

# 7. 添加依赖关系 (重点！)
# 这句必须单独写，告诉 CMake 先生成 cfg 文件，再编译我们的库
add_dependencies(sentry_chassis_controller 
  ${PROJECT_NAME}_gencfg
)

# 8. 安装规则
install(TARGETS sentry_chassis_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES sentry_chassis_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
