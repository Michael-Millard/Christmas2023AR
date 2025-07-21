cmake_minimum_required(VERSION 3.10)
project(ChristmasProject2023)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set custom OpenCV paths
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
set(OpenCV_INCLUDE_DIRS "/usr/local/include/opencv4")
set(OpenCV_LIBRARY_DIRS "/usr/local/lib")

# Find OpenCV
find_package(OpenCV REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})