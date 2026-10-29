# Motor de física Dynquora

[English version](README.md)

Dynquora es una biblioteca de simulación XPBD (Extended Position-Based Dynamics) CPU-first para C y C++. Este repositorio contiene un MVP funcional, no un motor de producción completo. «Dynquora» es un nombre provisional: no implica disponibilidad, registro ni exclusividad de marca.

## Alcance actual

El núcleo implementa partículas con posición y velocidad, masas inversas, gravedad, restricciones de distancia XPBD, substeps, iteraciones, capacidad fija y una ABI C11. No implementa colisiones, broadphase, mallas deformables, multihilo, SIMD especializado, GPU, serialización ni integraciones con Godot o Unreal. La API de allocator está reservada en la ABI; el MVP rechaza allocators personalizados con `DQ_UNSUPPORTED_BACKEND` hasta que el almacenamiento SoA sea compatible con allocators.

Los datos se almacenan internamente y se reservan al crear el contexto. Tras la reserva, `dq_step` no incrementa el almacenamiento. `dq_particles_acquire` presta una vista zero-copy; debe liberarse con la misma generación antes de añadir datos o ejecutar otro paso. Una misma instancia no es thread-safe: serializa todas sus llamadas desde el host.

## Compilación

Requiere CMake 3.20 y un compilador C++20/C11.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /tmp/dynquora-install
```

En Windows, usa `cmake -S . -B build -G Ninja` o el generador de Visual Studio y, a continuación, `cmake --build build --config Release`. `DQ_ENABLE_NATIVE_ARCH` está desactivada por defecto, al igual que los sanitizers y warnings-as-errors. No se activa `fast-math`.

### Visor gráfico opcional

El visor de diagnóstico usa raylib y está separado del motor. Tras instalar raylib (en macOS: `brew install raylib`), ejecútalo con:

```bash
cmake -S . -B build-viewer -DDQ_BUILD_VIEWER=ON
cmake --build build-viewer --parallel
./build-viewer/dq_viewer
```

`Space` pausa, `N` avanza un paso y `R` reinicia la escena de cuerda XPBD.

## Modelo físico y límites

Para cada restricción de distancia, el solver proyecta `C(x)=|x_a-x_b|-L=0` mediante XPBD: `Δλ=(-C-α~λ)/(w_a+w_b+α~)`, donde `α~=compliance/h²`. Los valores de entrada deben ser finitos; `dt > 0`, las masas inversas y el compliance deben ser no negativos. Se recomienda usar un timestep fijo y pequeño con substeps. En esta primera versión, se omiten las posiciones coincidentes para evitar una normal indefinida.

Consulta [docs/architecture.md](docs/architecture.md) para conocer las decisiones de diseño, los contratos, las pruebas y el roadmap. La cabecera pública es [include/dynquora/physics.h](include/dynquora/physics.h), y [examples/c/minimal.c](examples/c/minimal.c) muestra el ciclo completo.

## Licencia

Se propone Apache-2.0. Antes de publicar, un titular de los derechos de autor debe revisar `LICENSE` y `NOTICE`.
