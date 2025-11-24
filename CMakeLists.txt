cmake_minimum_required(VERSION 3.10)

project(CudaPlayground LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(TEST1 src/main.cpp)
#CUDA
#Habilita el compliador de CUDA (nvcc)
#project(CudaPlayground LANGUAGES CXX CUDA)

#El estandar de c++ para CUDA 
#set(CMAKE_CUDA_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

