cmake_minimum_required(VERSION 3.13)
project(gfold VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_BUILD_TYPE "Release")


find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

add_subdirectory(./p4_cpg_solver/c/)

add_executable(test_solver main.cpp)

target_include_directories(test_solver PUBLIC 
        ${cpg_include}  
        ${Python3_INCLUDE_DIRS})

target_link_libraries(test_solver 
    cpg
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)


add_executable(find_tf find_tf.cpp)

target_include_directories(find_tf PUBLIC 
        ${cpg_include}  
        ${Python3_INCLUDE_DIRS})

target_link_libraries(find_tf 
    cpg
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)

add_executable(find_tf_2 find_tf_2.cpp)

target_include_directories(find_tf_2 PUBLIC 
        ${cpg_include}  
        ${Python3_INCLUDE_DIRS})

target_link_libraries(find_tf_2 
    cpg
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17)

add_executable(kos_pingpong kos_pingpong.cpp)

target_include_directories(GFOLD_solver PUBLIC 
        ${cpg_include}  
        ${Python3_INCLUDE_DIRS})