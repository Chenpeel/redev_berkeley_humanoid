# ================================================================
# cmake -S ./ -B ./build/
# cmake --build ./build/ --target all
# ================================================================

cmake_minimum_required(VERSION 3.10)
project(humanoid)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

#set(TORCH_PATH "/home/tk/Documents/libtorch/")

#find_package(Torch REQUIRED PATHS ${TORCH_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0
)
FetchContent_MakeAvailable(yaml-cpp)


# ================================ main application ================================

add_executable(main
    src/main.cpp
    src/udp.c
    src/serializer.c
    src/imu.cpp
    src/socketcan.cpp
    src/motor_controller.cpp
    src/real_humanoid.cpp
)

set_property(TARGET main PROPERTY CXX_STANDARD 17)

target_include_directories(main PRIVATE src)

target_link_libraries(main yaml-cpp::yaml-cpp)


# ================================ test applications ================================

# Torch
add_executable(test-torch tests/test-torch.cpp)
target_link_libraries(test-torch "${TORCH_LIBRARIES}")
set_property(TARGET test-torch PROPERTY CXX_STANDARD 17)


# IMU
add_executable(test-imu tests/test-imu.cpp src/imu.cpp)
target_include_directories(test-imu PRIVATE src)
set_property(TARGET test-imu PROPERTY CXX_STANDARD 17)


# UDP
add_executable(test-udp tests/test-udp.cpp src/udp.c)
target_include_directories(test-udp PRIVATE src)
set_property(TARGET test-udp PROPERTY CXX_STANDARD 17)


# Low-level control protocol
add_executable(test-lowlevel-control tests/test-lowlevel-control.cpp)
target_include_directories(test-lowlevel-control PRIVATE src)
set_property(TARGET test-lowlevel-control PROPERTY CXX_STANDARD 17)


# Locomotion parity logic
add_executable(test-locomotion-parity tests/test-locomotion-parity.cpp)
target_include_directories(test-locomotion-parity PRIVATE src)
set_property(TARGET test-locomotion-parity PROPERTY CXX_STANDARD 17)


# Locomotion specification
add_executable(test-locomotion-specification tests/test-locomotion-specification.cpp)
target_include_directories(test-locomotion-specification PRIVATE src)
set_property(TARGET test-locomotion-specification PROPERTY CXX_STANDARD 17)


# YAML
add_executable(test-yaml tests/test-yaml.cpp)
target_include_directories(test-yaml PRIVATE src)
target_link_libraries(test-yaml yaml-cpp::yaml-cpp)
set_property(TARGET test-yaml PROPERTY CXX_STANDARD 17)
