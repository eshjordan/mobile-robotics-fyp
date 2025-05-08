cmake_minimum_required(VERSION 3.22.0)
project(cpp_epuck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} src/cpp_epuck.cpp)

# Configure any tests
if(BUILD_TESTING)
  ## Testing
  ament_auto_find_test_dependencies()

  # Create the test executable
  ament_auto_add_gmock(${PROJECT_NAME}_unit_test
    test/test_BaseRobotCommsModel.cpp
    test/test_RobotCommsModel.cpp
  )

  if (LINTER_TESTS)
    include(ProcessorCount)
    ProcessorCount(nproc)
    math(EXPR nproc "${nproc}-1")
    set(TEST_JOBS --jobs ${nproc})
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/../../.clang-format)
    ament_clang_tidy(CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/../../.clang-tidy TIMEOUT 3600 ${TEST_JOBS} ${CMAKE_CURRENT_BINARY_DIR})
    ament_cppcheck(
      LIBRARIES boost gnu googletest opencv2 posix
      EXCLUDE include/Csv.hpp test/test_Csv.cpp # Problems with analysing all possible templates
    )
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

