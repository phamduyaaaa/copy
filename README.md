# copy
```bash
cmake_minimum_required(VERSION 3.8)
project(cpp_stm32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm thư viện
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Tìm LibSerial (Dùng PkgConfig cho chắc ăn như bài trước)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 2. Tạo file thực thi
add_executable(stm32_node src/stm32_node.cpp)

# 3. Include directories
target_include_directories(stm32_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

# 4. Link thư viện
ament_target_dependencies(stm32_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(stm32_node ${LIBSERIAL_LIBRARIES})

# 5. Install
install(TARGETS stm32_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
