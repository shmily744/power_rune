cmake_minimum_required(VERSION 3.26)
project(power_rune)

set(CMAKE_CXX_STANDARD 14)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#Python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/include")
#遍历所有Cpp文件
file(GLOB_RECURSE CPP_FILES src/*.cpp)
add_executable(power_rune ${CPP_FILES})

target_link_libraries(power_rune Eigen3::Eigen ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
