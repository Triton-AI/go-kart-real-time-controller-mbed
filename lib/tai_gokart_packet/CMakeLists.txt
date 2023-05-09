cmake_minimum_required(VERSION 3.8)
project(tai_gokart_packet)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(GKC_PACKET_LIB_SRC
  src/gkc_packet_factory.cpp
  src/gkc_packets.cpp
)

set(GKC_PACKET_LIB_HEADERS
  include/tai_gokart_packet/gkc_packet_factory.hpp
  include/tai_gokart_packet/gkc_packets.hpp
  include/tai_gokart_packet/gkc_packet_subscriber.hpp
  include/tai_gokart_packet/gkc_packet_utils.hpp
  include/tai_gokart_packet/version.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GKC_PACKET_LIB_SRC}
  ${GKC_PACKET_LIB_HEADERS}
)

# testing
if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_gkc_packet.cpp)
  set(TEST_GKC_PACKET_EXE test_gkc_packet)
  ament_add_gtest(${TEST_GKC_PACKET_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_GKC_PACKET_EXE} ${PROJECT_NAME})
endif()

ament_auto_package()
