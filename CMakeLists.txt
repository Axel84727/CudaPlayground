cmake_minimum_required(VERSION 3.10)

# Define the main project and enable C++
# Keep CUDA commented out for now; we'll uncomment in the next step
project(CudaPlayground LANGUAGES CXX) 

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------
# 1. RAYLIB CONFIGURATION
# ----------------------------------------------------------------
find_package(raylib REQUIRED)

# ----------------------------------------------------------------
# 2. SOURCE FILES DEFINITION
# ----------------------------------------------------------------
set(SOURCE_FILES
    src/main.cpp
    src/physics/body.cpp 
    src/physics/world.cpp 
    src/sim/movementSystem.cpp 
    src/sim/collisionSystem.cpp
    src/sim/systemManager.cpp
)

# ----------------------------------------------------------------
# 3. CREATE THE EXECUTABLE
# ----------------------------------------------------------------
add_executable(CudaPlayground ${SOURCE_FILES})

# ----------------------------------------------------------------
# 4. INCLUDE DIRECTORIES AND LINK LIBRARIES
# ----------------------------------------------------------------

# Public include directories (simulation headers)
target_include_directories(CudaPlayground PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
)

# Link libraries (raylib, math, etc.) to the final executable
target_link_libraries(CudaPlayground PUBLIC 
    ${raylib_LIBRARIES} 
    m 
)

# Raylib include directories
target_include_directories(CudaPlayground PUBLIC ${raylib_INCLUDE_DIRS})


# ----------------------------------------------------------------
# 5. TESTS CONFIGURATION
# ----------------------------------------------------------------
add_subdirectory(tests)

# Ensure tests are discoverable by ctest even when using the build directory directly
enable_testing()
# Explicitly add test referencing the built test executable
add_test(NAME run_tests COMMAND ${CMAKE_BINARY_DIR}/tests/run_tests)

# ----------------------------------------------------------------
# Benchmark target (headless, no Raylib)
# ----------------------------------------------------------------
# Benchmark target (headless, no Raylib) - optional
# ----------------------------------------------------------------
option(BUILD_BENCHMARK "Build headless benchmark executable" ON)
if(BUILD_BENCHMARK)
    add_executable(benchmark
        tools/benchmark.cpp
        src/physics/body.cpp
        src/physics/world.cpp
        src/sim/movementSystem.cpp
        src/sim/collisionSystem.cpp
        src/sim/systemManager.cpp
    )

    target_include_directories(benchmark PUBLIC ${CMAKE_SOURCE_DIR}/include)

    # Do not link Raylib for benchmark (headless)
    target_link_libraries(benchmark PUBLIC m)
endif()


# ----------------------------------------------------------------
# 6. FUTURE CUDA CONFIGURATION (UNCOMMENT IN THE NEXT PHASE)
# ----------------------------------------------------------------
# # Enable the CUDA compiler (nvcc)
# project(CudaPlayground LANGUAGES CXX CUDA)

# # C++ standard for CUDA
# set(CMAKE_CUDA_STANDARD 17)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)