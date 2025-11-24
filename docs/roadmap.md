# CudaPlayground - Development Roadmap

A structured learning path for building a 2D physics engine from scratch, progressing from basic C++ to GPU-accelerated CUDA implementation.

---

## Phase 0: Initial Setup

**Goal:** Establish project structure, build system, and first "hello world" of the engine.

**Milestone 0 — `project: initial scaffold`**

- Create folder structure: `src/`, `include/`, `tests/`, `data/`, `docs/`
- Minimal CMake configuration that generates a `sim` executable
- README with project goals and build instructions (include "no local GPU required" note)
- `docs/roadmap.md` with this plan documented
- **Learning:** Basic CMake, repository management, commits, and documentation

---

## Phase 1: Entity Core and State

**Goal:** Represent entities and advance one simulation "tick" on CPU.

**Milestone 1 — `core: entities and world`**

- Design `Entity` structure (position, velocity, mass, radius or bbox)
- Implement `World` with list/vector of entities
- `main` that initializes N entities with random values and executes 1 tick (update)
- Document minimal API in `include/`
- **Learning:** Data design in C++, vectors, simple structs/classes

**Milestone 2 — `sim: basic loop and dt`**

- Implement main loop with fixed delta time (e.g., dt = 0.016s)
- Update position: `pos += vel * dt`
- Simple logging every N ticks
- **Learning:** Simulation loops, simple integrators, discrete time

---

## Phase 2: Collisions and Resolution (broad-phase → narrow-phase)

**Goal:** Basic collision detection and simple resolution.

**Milestone 3 — `physics: AABB/circles detection (broad-phase)`**

- Implement simple broad-phase detector (uniform grid or basic sweep & prune)
- Tests: small scenes where collisions are/aren't detected
- **Learning:** Structures to reduce complexity from O(n²) → O(n) or O(n log n)

**Milestone 4 — `physics: narrow-phase and response`**

- Implement simple response for circles (basic elastic impulse) or AABB push-out
- Resolve minimal penetration and adjust velocities (simple elastic collision)
- Add parameters: restitution, friction
- **Learning:** Collision physics, momentum conservation, mathematical response

---

## Phase 3: Integrators and Stability

**Goal:** Compare integrators and numerical stability.

**Milestone 5 — `integrators: euler vs semi-implicit euler vs verlet`**

- Implement at least 2 integrators (explicit and semi-implicit/Verlet)
- Create test/scenario where explicit Euler explodes and Verlet doesn't
- Document when to use each one
- **Learning:** Numerical integration, stability, tradeoffs

---

## Phase 4: Modular Structure and API

**Goal:** Separate modules and create API for adding systems.

**Milestone 6 — `refactor: modularize subsystems`**

- Separate folders: `physics/`, `math/`, `io/`, `sim/`
- Minimal interfaces: `ISystem::update(World&, dt)`
- Add `PhysicsSystem` and `MovementSystem`
- **Learning:** Modular design, separation of concerns, interfaces in C++

---

## Phase 5: Minimal Visualization (optional but useful)

**Goal:** See what's happening without debugging through prints.

**Milestone 7 — `viz: simple rendering with SDL2 or raylib`**

- Integrate basic rendering that draws circles/rects on screen
- Option: headless mode that dumps state every N ticks to `data/frames/`
- **Learning:** Integration with C libraries, render vs sim loop, headless testing

---

## Phase 6: Profiling and Benchmarks

**Goal:** Measure hotspots and think about parallelization.

**Milestone 8 — `perf: timers and benchmarks`**

- Add CPU timers (`std::chrono`) to measure: total update, broad-phase, narrow-phase, resolve
- Script `docs/benchmarking.md` describing how to run benchmarks and what to measure
- Execute with various N sizes (100, 1k, 10k) and save results
- **Learning:** Simple benchmarking, identifying bottlenecks

---

## Phase 7: Prepare for Parallelization (GPU design)

**Goal:** Reorganize data for "data-parallel" pattern.

**Milestone 9 — `design: data layout for GPU`**

- Transform `World` to SoA (Structure of Arrays): arrays of posX[], posY[], velX[], velY[], mass[]
- Document how computation would map to kernels (which parts are embarrassingly parallel)
- **Learning:** Memory-friendly layout, coalesced access, mental preparation for CUDA

---

## Phase 8: CPU Parallel Implementation (conceptual step)

**Goal:** Use threads to simulate intermediate step before CUDA.

**Milestone 10 — `parallel: CPU multithreading (OpenMP/std::thread)`**

- Parallelize force calculation phase or broad-phase with `std::thread` or OpenMP
- Verify with benchmarks whether it scales (or not) and document issues (data races, sync)
- **Learning:** Concurrency, race conditions, synchronization

---

## Phase 9: First GPU Kernel (when GPU access is available)

**Goal:** Port one or two cores to CUDA.

**Milestone 11 — `cuda: positions update kernel`**

- Trivial kernel: update positions from velocities (data-parallel)
- H2D/D2H copies, GPU vs CPU timing measurement
- **Learning:** Host-device flow, memory transfer, basic kernel

**Milestone 12 — `cuda: parallel broad-phase`**

- Implement parallel broad-phase (e.g., spatial grid hashing) on GPU
- **Learning:** Parallelism patterns, atomic ops, reductions

---

## Phase 10: Advanced GPU Optimization

**Goal:** Profiling, coalescing, bank conflicts, and tuning.

**Milestone 13 — `cuda: optimization and profiling`**

- Resolve bank conflicts in shared memory
- Reorder data for coalescing
- Measure and document improvements
- **Learning:** Memory coalescing, shared memory, occupancy, profiling with Nsight (or describe how you would do it)
