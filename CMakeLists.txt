cmake_minimum_required(VERSION 3.14)
project(neuro_control_project)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force CMake to use the exact Python 3.12 environment installed on your Mac
set(Python3_EXECUTABLE "/Library/Frameworks/Python.framework/Versions/3.12/bin/python3")

# Find Python3 and NumPy automatically
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# Create the executable
add_executable(neuro_sim main.cpp)

# Disable the noisy Python 3.12 deprecation warnings
target_compile_options(neuro_sim PRIVATE -Wno-deprecated-declarations)

# Link the Python and NumPy include directories
target_include_directories(neuro_sim PRIVATE 
    ${Python3_INCLUDE_DIRS} 
    ${Python3_NumPy_INCLUDE_DIRS}
)

# Link the Python libraries
target_link_libraries(neuro_sim PRIVATE ${Python3_LIBRARIES})
