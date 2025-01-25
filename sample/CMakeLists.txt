cmake_minimum_required(VERSION 3.10)

# プロジェクト名とC++標準
project(MuJoCoSample)
set(CMAKE_CXX_STANDARD 17)

# MuJoCoのヘッダーファイルとライブラリのパスを指定
include_directories(${CMAKE_SOURCE_DIR}/../include)
link_directories(${CMAKE_SOURCE_DIR}/../lib)

MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

# 実行ファイルを指定
add_executable(main src/main.cpp)

# MuJoCoライブラリをリンク
target_link_libraries(main mujoco)

# GLFWを必要とする場合（MuJoCoと一緒に使用するため）
find_package(glfw3 REQUIRED)
target_link_libraries(main glfw)
