cmake_minimum_required(VERSION 3.16)

# 主机侧（x86 Linux/Windows）单元测试：不依赖 STM32 工具链，也不参与固件镜像。
# 交叉编译固件时这一层不会被包含（见上级 CMakeLists.txt 的条件 include）。
project(infantry_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(hipnuc_crc_test hipnuc_crc_test.cpp)
target_include_directories(hipnuc_crc_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../STM32F405"
)

enable_testing()
add_test(NAME hipnuc_crc COMMAND hipnuc_crc_test)
