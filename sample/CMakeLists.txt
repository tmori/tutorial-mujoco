cmake_minimum_required(VERSION 3.10)

# プロジェクト名とC++標準
project(MuJoCoSample)
set(CMAKE_CXX_STANDARD 17)


set(HAKO_INC_DIR "/usr/local/include/hakoniwa")

# MuJoCoのヘッダーファイルとライブラリのパスを指定
include_directories(${CMAKE_SOURCE_DIR}/../include)
include_directories(${CMAKE_SOURCE_DIR}/../../../hakoniwa-ros2pdu/pdu/types)
link_directories(${CMAKE_SOURCE_DIR}/../lib)
link_directories(/usr/local/lib/hakoniwa)

MESSAGE(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

# 実行ファイルを指定
add_executable(main src/main.cpp)

target_include_directories(
    main
    PRIVATE ${HAKO_INC_DIR}
)
# MuJoCoライブラリをリンク
target_link_libraries(main 
    assets
    conductor
    mujoco)

# GLFWを必要とする場合（MuJoCoと一緒に使用するため）
find_package(glfw3 REQUIRED)
target_link_libraries(main glfw)
