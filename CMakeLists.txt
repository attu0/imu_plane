cmake_minimum_required(VERSION 3.5)
project(imu_plane)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ✅ Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ========================
# Install resource folders
# ========================
install(
  DIRECTORY 
    config 
    description 
    launch 
    worlds
    meshes
    scripts
  DESTINATION share/${PROJECT_NAME}
)

# ========================
# Install Python executables
# ========================
install(PROGRAMS
  scripts/imu_node.py
  scripts/tf_broadcaster_imu.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# ✅ Install Python module (FIX)
# ========================
ament_python_install_package(mpu_6050_driver)

# ========================
# Testing (optional)
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()