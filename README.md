# CudaPlayground

A 2D physics engine built from scratch to learn C++ and CUDA.

## Goals

- Learn modern C++ through hands-on project
- Understand physics simulation fundamentals
- Master CUDA parallel programming
- Build something that scales to millions of particles

## Current Status

Work in Progress - Phase 4

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

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
./CudaPlayground
```

If CMake can't find Raylib, set `raylib_DIR` or `CMAKE_PREFIX_PATH` to the location where raylib was installed.

Runtime controls (useful keybindings implemented in the simulator):

- P : Pause / Resume simulation
- N : Advance one physics step (when paused)
- O : Save snapshot (in-memory)
- L : Load previously saved snapshot
- SPACE : Spawn a new body at mouse position
- 1 / 2 : Decrease / Increase spawn mass
- 3 / 4 : Decrease / Increase spawn restitution
- 5 / 6 : Decrease / Increase spawn radius
- M / B : Increase / Decrease mass of selected body
- R / T : Increase / Decrease restitution of selected body
- S / A : Increase / Decrease radius of selected body

Notes

- The simulation uses a fixed time step for physics (1/60s) and a positional Verlet integrator.
- If you change `GridInfo` bounds or `world_scale`, adjust the window mapping accordingly.

## Learning Journey

This is my first serious C++/CUDA project. The goal is to learn by building something real and useful.

## License

MIT
