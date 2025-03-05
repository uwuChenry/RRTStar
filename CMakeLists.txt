cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)  # Use C++17

# Add include directory
include_directories(include)

# Find all .cpp files in src and add them as sources
file(GLOB SOURCES "src/*.cpp")

# Create the executable
add_executable(my_program ${SOURCES})

