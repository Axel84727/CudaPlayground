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
    src/math/vec2.cpp 
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


# ----------------------------------------------------------------
# 6. FUTURE CUDA CONFIGURATION (UNCOMMENT IN THE NEXT PHASE)
# ----------------------------------------------------------------
# # Enable the CUDA compiler (nvcc)
# project(CudaPlayground LANGUAGES CXX CUDA)

# # C++ standard for CUDA
# set(CMAKE_CUDA_STANDARD 17)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)