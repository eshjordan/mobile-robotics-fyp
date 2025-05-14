cmake_minimum_required(VERSION 3.22.0)
project(agent_common_library)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Define the library name and target files
file(GLOB SOURCES "src/*.cpp")
ament_auto_add_library(agent_common_library
  SHARED ${SOURCES}
)

# Enable all warnings and treat them as errors
target_compile_options(agent_common_library PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Configure any tests
if(BUILD_TESTING)

  ## Testing
  ament_auto_find_test_dependencies()

  # Source files for test
  file(GLOB TEST_SOURCES
    "test/*.cpp"
  )

  # Create the test executable
  ament_auto_add_gmock(${PROJECT_NAME}_unit_test
    ${TEST_SOURCES}
  )
  target_link_libraries(${PROJECT_NAME}_unit_test
    agent_common_library
  )

  ## Linting
  if(ENABLE_LINTING)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/../../.clang-format)
    ament_clang_tidy(CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/../../.clang-tidy TIMEOUT 3600 ${CMAKE_CURRENT_BINARY_DIR})
    ament_cppcheck(
      LIBRARIES boost gnu googletest opencv2 posix
      EXCLUDE include/Csv.hpp test/test_Csv.cpp # Problems with analysing all possible templates
    )
  endif()

endif()

# Add the tools directory to the PATH so that its scripts can be run once the package is built and sourced.
# This will run clangd-tidy (much faster) instead of the full clang-tidy.
set(EXTRA_PATHS "${CMAKE_CURRENT_LIST_DIR}/../../tools/clang-tidy")
ament_environment_hooks(
  cmake/env/add_to_path.sh.in
  cmake/env/add_to_path.bash.in
  cmake/env/enable_cppcheck.sh.in
  cmake/env/enable_cppcheck.bash.in
)

# Enable running cppcheck with ament_cppcheck() if it is auto-disabled.
ament_environment_hooks(
  cmake/env/enable_cppcheck.sh.in
  cmake/env/enable_cppcheck.bash.in
)

# Install the library so that it can be used by other packages
install(
  DIRECTORY include/
  DESTINATION include/agent_common_library
)

# Build the package
ament_auto_package(
  INSTALL_TO_SHARE
    include
)
