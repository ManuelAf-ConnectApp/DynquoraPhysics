# Dynquora Physics Engine

[Versión en español](README.es.md)

Dynquora is a CPU-first XPBD (Extended Position-Based Dynamics) simulation library for C and C++. This repository contains a functional MVP, not a complete production engine. “Dynquora” is a provisional name and does not imply name availability, registration, or trademark exclusivity.

## Current scope

The core implements particles with position and velocity, inverse masses, gravity, XPBD distance constraints, a compact 3D uniform-grid broadphase, particle contacts, a floor plane, substeps, iterations, fixed capacity, and a C11 ABI. It does not implement deformable meshes, multithreading, specialized SIMD, GPU support, serialization, or Godot/Unreal integrations. The allocator API is reserved in the ABI; this MVP rejects custom allocators with `DQ_UNSUPPORTED_BACKEND` until the SoA storage becomes allocator-aware.

Data is stored internally and allocated when the context is created. Once allocated, `dq_step` does not grow storage. `dq_particles_acquire` creates a preallocated AoS read snapshot from the internal SoA buffers; it must be released using the same generation before adding data or running another step. A single instance is not thread-safe: serialize all calls from the host. Particle contacts require `grid_cell_size >= 2 * particle_radius`.

## Building

Requires CMake 3.20 and a C++20/C11 compiler.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /tmp/dynquora-install
```

On Windows, use `cmake -S . -B build -G Ninja` or the Visual Studio generator, then `cmake --build build --config Release`. `DQ_ENABLE_NATIVE_ARCH` is disabled by default, as are sanitizers and warnings-as-errors. `fast-math` is not enabled.

### Optional graphical viewer

The diagnostic viewer uses raylib and is separate from the engine. After installing raylib (on macOS: `brew install raylib`), run:

```bash
cmake -S . -B build-viewer -DDQ_BUILD_VIEWER=ON
cmake --build build-viewer --parallel
./build-viewer/dq_viewer
```

`Space` pauses, `N` advances one step, and `R` resets the XPBD rope scene.

## Physical model and limits

For every distance constraint, the solver projects `C(x)=|x_a-x_b|-L=0` using XPBD: `Δλ=(-C-α~λ)/(w_a+w_b+α~)`, where `α~=compliance/h²`. Input values must be finite; `dt > 0`, inverse masses, and compliance must be non-negative. A small fixed timestep with substeps is the recommended mode. Coincident positions are skipped in this first version to avoid an undefined normal.

See [docs/architecture.md](docs/architecture.md) for design decisions, contracts, tests, and the roadmap. The public header is [include/dynquora/physics.h](include/dynquora/physics.h), and [examples/c/minimal.c](examples/c/minimal.c) demonstrates the full cycle.

## License

Apache-2.0 is proposed. Before publishing, a copyright holder should review `LICENSE` and `NOTICE`.
