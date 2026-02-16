cmake_minimum_required(VERSION 3.13)
project(gfold VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
endif()


find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

# ===== CVXPYgen solver (single resolution) =====
# Generated solver now lives at p4_cpg_solver/c (no per-N variants).
set(CPG_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/p4_cpg_solver/c")
if(NOT EXISTS "${CPG_SOLVER_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "CPG solver dir not found: ${CPG_SOLVER_DIR}. Run solver.py --gen to regenerate.")
endif()
add_subdirectory(${CPG_SOLVER_DIR})

add_executable(test_solver main.cpp GFOLD_solver.cpp)
add_executable(GFOLD_demo GFOLD_demo.cpp GFOLD_solver.cpp find_tf_2.cpp)

target_include_directories(test_solver PUBLIC 
        ${cpg_include}  
        ${CPG_SOLVER_DIR}/solver_code
        ${Python3_INCLUDE_DIRS})

target_link_libraries(test_solver 
    cpg
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)

target_include_directories(GFOLD_demo PUBLIC 
        ${cpg_include}  
        ${CPG_SOLVER_DIR}/solver_code
        ${Python3_INCLUDE_DIRS})

target_link_libraries(GFOLD_demo 
    cpg)


add_executable(find_tf find_tf.cpp GFOLD_solver.cpp)

target_include_directories(find_tf PUBLIC 
        ${cpg_include}  
        ${CPG_SOLVER_DIR}/solver_code
        ${Python3_INCLUDE_DIRS})

target_link_libraries(find_tf 
    cpg
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)

add_executable(kos_pingpong kos_pingpong.cpp)
