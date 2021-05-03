cmake_minimum_required(VERSION 3.12)
project(diffDriveRobotSim LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "build/")

include_directories("include/" "/usr/include/python3.8")


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(main ${SRCS})
target_link_libraries(main Eigen3::Eigen python3.8 ${OpenCV_LIBS})