# Dynquora Physics Engine

Dynquora es una biblioteca de simulación XPBD (Extended Position-Based Dynamics) CPU-first para C y C++. Este repositorio contiene un MVP funcional, no un motor de producción completo. «Dynquora» es un nombre provisional: no implica disponibilidad, registro ni exclusividad de marca.

## Alcance actual

El núcleo implementa partículas con posición/velocidad, masas inversas, gravedad, restricciones de distancia XPBD, substeps, iteraciones, capacidad fija y una ABI C11. No implementa colisiones, broadphase, malla deformable, multihilo, SIMD especializado, GPU, serialización ni integraciones Godot/Unreal. La API de allocator está reservada en la ABI; el MVP rechaza allocators personalizados con `DQ_UNSUPPORTED_BACKEND` hasta que el almacenamiento SoA sea allocator-aware.

Los datos se almacenan internamente y se reservan al crear el contexto. Tras la reserva no hay crecimiento en `dq_step`. `dq_particles_acquire` presta una vista zero-copy; debe liberarse con la misma generación antes de añadir datos o ejecutar otro paso. La misma instancia no es thread-safe: serialice todas sus llamadas en el host.

## Compilación

Requiere CMake 3.20 y compilador C++20/C11. No se verificó CMake en el entorno de desarrollo de este cambio porque no estaba instalado; la librería, ejemplo C y test sí se comprobaron directamente con Clang.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /tmp/dynquora-install
```

En Windows, use `cmake -S . -B build -G Ninja` o el generador Visual Studio y `cmake --build build --config Release`. `DQ_ENABLE_NATIVE_ARCH` está desactivada por defecto, como también sanitizers y warnings-as-errors. No se activa `fast-math`.

### Visor gráfico opcional

El visor técnico usa raylib y está separado del motor. Tras instalar raylib (en macOS: `brew install raylib`), ejecútelo con:

```bash
cmake -S . -B build-viewer -DDQ_BUILD_VIEWER=ON
cmake --build build-viewer --parallel
./build-viewer/dq_viewer
```

`Space` pausa, `N` avanza un paso y `R` reinicia la escena de cuerda XPBD.

## Modelo físico y límites

Para cada restricción de distancia, el solver proyecta `C(x)=|x_a-x_b|-L=0` mediante XPBD: `Δλ=(-C-α~λ)/(w_a+w_b+α~)`, con `α~=compliance/h²`. Los valores admitidos son finitos; `dt > 0`, masas inversas y compliance no negativos. Un timestep fijo y pequeño, con substeps, es el modo recomendado. Las posiciones coincidentes se omiten en esta primera versión para evitar una normal indefinida.

Véase [docs/architecture.md](docs/architecture.md) para decisiones, contratos, pruebas y roadmap. La cabecera pública es [include/dynquora/physics.h](include/dynquora/physics.h), y [examples/c/minimal.c](examples/c/minimal.c) muestra el ciclo completo.

## Licencia

Apache-2.0 propuesta. Antes de publicar, un titular de copyright debe revisar `LICENSE` y `NOTICE`.
