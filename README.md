# Physix2D

A 2D physics engine built from scratch to learn C++ and CUDA.

## Goals

- Learn modern C++ through a hands-on project  
- Understand physics simulation fundamentals (collision, impulse resolution, damping, restitution)  
- Build a modular and extensible engine  
- Simulate many interacting bodies efficiently 

## Current Status

Work finished

this was the roadmap that I use to create this (the origninal plan was to add cuda but I do not have a Nvidia GPU)
See [detailed roadmap](docs/roadmap.md) for full development plan.

## Building

This project now uses Raylib for simple rendering. Before building, install the Raylib dependency for your platform.

macOS (Homebrew):

```bash
# Install raylib
brew install raylib
# Install cmake if you don't have it
brew install cmake
```

Ubuntu / Debian:

```bash
# Install dependencies (example)
sudo apt update
sudo apt install build-essential cmake libraylib-dev
```

General build steps (from project root):

Note: run the following commands from the repository root (where this README.md lives). Replace the path to the repo if it's different on your machine.

```bash
# Configure an out-of-source build and generate build files
cmake -S . -B build

# Build using the generated build system (parallel)
cmake --build build -j

# Run the graphical executable
./build/CudaPlayground
```

If you prefer the older style (entering the build dir) you can also do:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
../build/CudaPlayground || ./CudaPlayground
```

If CMake can't find Raylib, set `raylib_DIR` or `CMAKE_PREFIX_PATH` to the location where raylib was installed.

If your checkout lives in a different path than the examples above, just `cd` into the project root first and run the same commands — the build is relative to the repo root, so it works on other machines without path edits.

Controls (keys and interactions)

The following keys and mouse actions are implemented in the simulator (useful both for the visual demo and local testing):

- P : Pause / Resume the simulation
- N : Advance one physics step while paused
- O : Save an in-memory snapshot
- L : Load the previously saved snapshot
- Space : Spawn a new body at the mouse cursor position
- 1 / 2 : Decrease / Increase default spawn mass
- 3 / 4 : Decrease / Increase default spawn restitution
- 5 / 6 : Decrease / Increase default spawn radius
- Left click : Select a body
- Click and drag (left) : Drag / throw a body. Release to apply velocity based on mouse history
- Mouse extra buttons (non-standard): may be supported depending on platform
- M / B : Increase / Decrease mass of the selected body
- R / T : Increase / Decrease restitution of the selected body
- S / A : Increase / Decrease radius of the selected body
- D / F : Increase / Decrease damping (air resistance) of the selected body
- G / H : Increase / Decrease friction coefficient of the selected body
- Delete / X : Remove the selected body

Interaction notes:

- When impulses are applied (collisions, throwing a body, or direct velocity edits) the Verlet integrator synchronizes the previous position internally to keep simulation consistent.
- Bodies bounce using impulse resolution plus positional correction to avoid penetration (slop + percent). Each body has its own restitution.
- If you change `GridInfo` or `world_scale`, remember to keep world-to-window mapping consistent.

Notes

- The simulation uses a fixed time step for physics (1/60s) and a positional Verlet integrator.
- If you change `GridInfo` bounds or `world_scale`, adjust the window mapping accordingly.

## Learning Journey

This is my first serious C++/CUDA project. The goal is to learn by building something real and useful.

## License

MIT
