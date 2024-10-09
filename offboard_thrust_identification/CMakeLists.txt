cmake_minimum_required(VERSION 3.8)
project(offboard_thrust_identification)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

find_package(ament_cmake REQUIRED)
find_package(uav_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(thrust_matcher thrust_matcher.cpp)
target_include_directories(thrust_matcher PUBLIC include)

ament_target_dependencies(thrust_matcher uav_cpp)

install(TARGETS
    thrust_matcher
    DESTINATION lib/${PROJECT_NAME}
)
