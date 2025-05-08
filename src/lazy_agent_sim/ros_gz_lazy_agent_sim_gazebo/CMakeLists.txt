cmake_minimum_required(VERSION 3.22.0)
project(ros_gz_lazy_agent_sim_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND KNOWN_GZ_VERSIONS
  "garden"
  "harmonic"
  "ionic"
)

# Set default Gazebo version, or use the one set in the environment variable
if ("$ENV{GZ_VERSION}" IN_LIST KNOWN_GZ_VERSIONS)
  set(GZ_VERSION "$ENV{GZ_VERSION}")
else()
  set(GZ_VERSION "ionic")
endif()

add_library(gz-sim INTERFACE)

# Garden
if("${GZ_VERSION}" STREQUAL "garden")
  message(STATUS "Compiling against Gazebo Garden")
  find_package(gz-sim7 REQUIRED)
  target_link_libraries(gz-sim INTERFACE gz-sim7::gz-sim7)
# Harmonic
elseif("${GZ_VERSION}" STREQUAL "harmonic")
  message(STATUS "Compiling against Gazebo Harmonic")
  find_package(gz-sim8 REQUIRED)
  target_link_libraries(gz-sim INTERFACE gz-sim8::gz-sim8)
# Ionic
elseif("${GZ_VERSION}" STREQUAL "ionic")
  message(STATUS "Compiling against Gazebo Ionic")
  find_package(gz-sim9 REQUIRED)
  target_link_libraries(gz-sim INTERFACE gz-sim9::gz-sim9)
endif()


# ament_auto_add_library(BasicSystem SHARED
#   src/BasicSystem.cc
# )

# target_link_libraries(BasicSystem
#   gz-sim
# )


# ament_auto_add_library(FullSystem SHARED
#   src/FullSystem.cc
# )

# target_link_libraries(FullSystem
#   gz-sim
# )

# Following directives are used when testing.
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
endif()


# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_auto_package(
  INSTALL_TO_SHARE
    worlds
)
