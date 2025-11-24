cmake_minimum_required(VERSION 3.10)

project(CudaPlayground LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(SOURCE_FILES
    src/main.cpp
    src/vec2.cpp
    src/body.cpp
    src/world.cpp
)

add_executable(TEST1 ${SOURCE_FILES})

target_include_directories(TEST1 PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
)
#CUDA
#Habilita el compliador de CUDA (nvcc)
#project(CudaPlayground LANGUAGES CXX CUDA)

#El estandar de c++ para CUDA 
#set(CMAKE_CUDA_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

