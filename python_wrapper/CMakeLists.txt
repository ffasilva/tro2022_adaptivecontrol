cmake_minimum_required(VERSION 3.15...3.26)

project(adaptive_control_example_python_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type -fPIC)

if(APPLE)
    include_directories(
            /usr/local/include/
            /usr/local/include/eigen3
            /opt/homebrew/include
            /opt/homebrew/include/eigen3
    )
endif()

add_subdirectory(submodules/dqrobotics/python/pybind11)
pybind11_add_module(_core MODULE
    src/adaptive_control_example_py.cpp

    )

set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(cpp_version)
target_link_libraries(_core PRIVATE adaptive_control)

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})