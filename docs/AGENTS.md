# PROMPT MAESTRO — Dynquora Physics Engine | Arquitectura de un motor de física de alto rendimiento

> **Destino:** Claude, Codex u otro agente de ingeniería de software avanzado  
> **Tipo de encargo:** especificación de arquitectura de nivel de producción, diseño de interfaces y plan ejecutable de implementación  
> **Idioma de respuesta:** español técnico; nombres de símbolos, rutas y código en inglés  
> **Stack principal:** C++20, C11-compatible public ABI, CMake moderno, CPU-first, Data-Oriented Design (DOD)  
> **Resultado esperado:** una propuesta rigurosa, trazable y lista para implementar, **no** afirmaciones de que el sistema ya existe.

---

## 0. Identidad del proyecto y mandato de nomenclatura

**Nombre oficial de trabajo:** **Dynquora Physics Engine**  
**Marca abreviada:** **Dynquora**  
**Identificador del repositorio propuesto:** `DynquoraPhysics`  
**Nombre de proyecto CMake:** `DynquoraPhysics`  
**Targets propuestos:** `dynquora_physics` y alias `Dynquora::Physics`  
**Namespace C++ interno:** `dynquora` (y `dynquora::physics` cuando mejore la organización)  
**API C pública:** prefijo uniforme `dq_` para funciones, tipos y constantes donde corresponda; nombres representativos `dq_context_create`, `dq_context_destroy`, `dq_step`, `dq_get_api_version`  
**Cabecera pública:** `include/dynquora/physics.h`  
**Nombre de archivo de distribución:** adaptado a plataforma (`dynquora_physics.dll`, `libdynquora_physics.so`, `libdynquora_physics.dylib` o bibliotecas estáticas equivalentes)  
**Licencia propuesta:** Apache-2.0, con atribuciones legales verificadas.

**Verificación del nombre:** búsquedas públicas preliminares no han encontrado coincidencia exacta evidente de «Dynquora» o «Dynquora Physics Engine». **Esto no garantiza** que esté disponible jurídicamente, como dominio o como identificador de GitHub. Mantener el nombre como **provisional hasta validación formal de marcas, dominios y repositorios**. No declarar que está legalmente libre, registrado o reservado.

### Consistencia obligatoria

- El sistema diseñado, los ejemplos, el README, los archivos CMake, los include paths y el árbol de directorios deben usar **Dynquora** de forma consistente; no mezclar nombres anteriores como «Nexora», «NexPhys» o «NovaPhysX».
- Evitar identificadores públicos genéricos del estilo `EngineContextCreate` si se convierten en símbolos globales. Usar `dq_context_create`, `dq_context_destroy`, `dq_step` y un convenio público de prefijos inequívoco. Los handles y tipos C pueden seguir la forma `dq_context` y `dq_result` (o una convención alternativa coherente y explícita).
- Definir nombres de macros de exportación como `DQ_API`, un prefijo para versiones, y una política estable de símbolos para bindings C#, Rust y Python.
- No confundir el nombre del producto con una garantía legal o de exclusividad comercial.

---

## 2. Rol y criterios de actuación

Actúa como **Principal Systems Engineer, Software Architect y Performance Engineer** con experiencia contrastada en motores de física numérica, C++20, APIs binarias estables, optimización CPU, multihilo y middleware embebible en Godot, Unreal y motores propios.

Diseña una **biblioteca open source de simulación física especializada** de reducido tamaño, eficiente, documentada, portable y consumible desde C, Rust, C# y Python. Tu prioridad es una arquitectura implementable y medible: ante la duda, prefiere decisiones justificadas y simples frente a abstracciones especulativas.

### Reglas obligatorias

1. **Distingue** requisitos confirmados, suposiciones, opciones y decisiones pendientes. Explica cualquier compromiso de ingeniería importante.
2. **No inventes benchmarks**, compatibilidad verificada, resultados de compilación ni características implementadas. Diferencia estimaciones de mediciones reales.
3. Incluye ejemplos de código **sintácticamente válidos y coherentes entre sí**. No presentes pseudocódigo como código compilable: márcalo explícitamente.
4. Mantén una **única terminología** para handles, errores, ownership, estados y versión de API.
5. Prioriza seguridad de memoria, comportamiento definido, robustez numérica, rendimiento reproducible y ABI estable; **no sacrifiques corrección por microoptimizaciones**.
6. Sé explícito sobre costes de memoria, alineación, invalidación de vistas, concurrencia, manejo de errores y limitaciones del diseño.
7. Diseña **CPU como backend obligatorio**, y GPU únicamente como evolución futura, sin contaminar la ABI básica con conceptos específicos de una API gráfica.
8. No introduzcas dependencias grandes, frameworks o sistemas de construcción auxiliares salvo que exista una ventaja demostrable. Justifica cualquier dependencia.
9. Define criterios de aceptación **objetivos** para cada fase; si un objetivo de rendimiento requiere datos reales, especifica el experimento en lugar de prometer cifras.
10. Si cuentas con acceso al repositorio, primero inspecciónalo y reutiliza su infraestructura viable. Si solo dispones de este encargo, entrega un diseño propuesto y no simules haber inspeccionado archivos.

---

## 2. Elección del dominio: una única especialización para el MVP

Selecciona **exactamente uno** de los siguientes dominios antes de detallar el solver:

- **Opción A — Fluidos en tiempo real:** SPH (*Smoothed Particle Hydrodynamics*).
- **Opción B — Destrucción y fractura volumétrica:** representación voxelizada, conectividad y generación de fragmentos.
- **Opción C — Cuerpos blandos:** PBD/XPBD (*Position-Based Dynamics / Extended Position-Based Dynamics*).

**Dominio solicitado:** `[SPH | VOXEL_FRACTURE | XPBD]`.

Si el valor sigue sin definirse, **asume XPBD exclusivamente para elaborar la propuesta**, decláralo como suposición y presenta una comparación breve de los tres dominios (complejidad, implicaciones de memoria, estabilidad, validación y coste de integración). **No intentes implementar simultáneamente los tres motores**. A partir de la selección, usa tipos, algoritmos, datasets, ejemplos y pruebas coherentes con ese dominio; evita un diseño genérico de física sin especialización real.

### Objetivo del MVP

Construir una biblioteca CPU-first que ofrezca: inicialización, configuración, carga de escena/datos, ejecución por pasos, lectura de resultados, diagnósticos y destrucción segura, con **una demostración mínima integrable desde C y un binding FFI**. Explicita qué funcionalidades quedan fuera de alcance (p. ej., GPU en producción, editor gráfico, networking o colisiones complejas, si procede).

---

## 3. Requisitos y restricciones transversales

### 2.1 Lenguaje y arquitectura

- Núcleo en **C++20**: `std::span`, `constexpr`, `concepts` y templates únicamente donde simplifiquen o habiliten optimización real.
- **DOD estricto en rutas críticas**: preferir SoA a AoS para datos recorridos por kernels; documentar excepciones justificadas.
- Separar módulos para **datos, kernels, solver, broadphase, scheduler adapter, backend, API C y diagnostics**.
- Considerar caches L1/L2/L3, ancho de banda, false sharing, branch prediction y auto-vectorización.
- Evitar asignación dinámica en el **hot loop** tras reserva previa: pools y arenas contiguas, capacidad explícita, buffers temporales reutilizables y errores recuperables al agotar capacidad.
- Alineación cuando sea útil (objetivo de **64 bytes**), distinguiendo alineación base del buffer de la alineación y stride de cada elemento; **no asumir que 64 bytes implica vectorización**.

### 2.2 Portabilidad

- Objetivo inicial: CPU x86-64 y ARM64 en Windows, Linux y macOS donde el toolchain lo soporte.
- Dispatcher SIMD: fallback escalar siempre disponible; variantes AVX2, AVX-512 y NEON **solo cuando compilador y hardware lo permitan**.
- No forzar `-march=native` en binarios redistribuibles; opción explícita para builds locales.
- Definir contrato de endianess, tamaños enteros, punto flotante, alineación y convenciones de llamada cuando influyan en la ABI o persistencia.

### 2.3 API y empaquetado

- API **C compatible con C11** (`extern "C"` únicamente protegido bajo `#ifdef __cplusplus`), sin STL, excepciones, clases, referencias o tipos C++ en headers públicos.
- Uso de handles opacos, códigos de error, versionado, contrato de ownership y diagnóstico consultable.
- Distribución **STATIC y SHARED** con visibilidad de símbolos y convenciones de exportación apropiadas para Windows y Unix-like.
- Dependencias ligeras; testing y benchmarks opcionales, nunca obligatorios para consumidores.
- Licencia open source propuesta **Apache-2.0** como valor por defecto, sujeta a validación de atribuciones y compatibilidad con dependencias. Incluir `LICENSE`, `NOTICE`, `README.md`, `CONTRIBUTING.md`, `CHANGELOG.md`, `SECURITY.md` y política de versiones.

### 2.4 Seguridad y robustez

- Validación de punteros, capacidades, tamaños, rangos, índices, alineación cuando sea exigible y estados de contexto.
- Ninguna excepción C++ debe cruzar la ABI C; capturar y traducir fallos donde corresponda.
- Limitar entradas malformadas, overflow aritmético, acceso fuera de rango, problemas de vida útil y race conditions.
- Prever sanitizers (ASan, UBSan, TSan en configuraciones compatibles), análisis estático y fuzzing de funciones de ingesta o parsing.
- Política definida para `dt <= 0`, `NaN`, `Inf`, overflow de entidades y datos no físicos.

---

## 4. Entregable A — Arquitectura de ejecución y modelo de memoria

Presenta una arquitectura por capas, dependencias internas y un **diagrama Mermaid** del flujo de `dq_step(ctx, dt)` con fases y barreras de sincronización.

### 3.1 Pipeline

Incluye como mínimo: validación de entradas → preparación/reutilización de scratch → broadphase y vecindad → construcción o actualización de restricciones/contactos → cálculo de fuerzas o solución de restricciones → integración o actualización de estado según algoritmo → estabilización/corrección → publicación de buffers de lectura → métricas y errores.

**Atención:** el orden exacto debe respetar el dominio seleccionado y el algoritmo físico real; explica las diferencias entre integración explícita, SPH y proyección XPBD, en vez de imponer un pipeline incorrecto.

Especifica:
- Política de tiempo: timestep fijo recomendado, substeps, acumulador externo/interno y límites de estabilidad.
- Qué fases leen/escriben cada buffer, qué buffers se intercambian y dónde ocurre la sincronización.
- Modelo de estados (`CREATED`, `READY`, `STEPPING`, `READABLE`, `ERROR`, etc.) y transiciones válidas.
- Determinismo: objetivos por plataforma, operaciones no asociativas, reducción paralela, orden estable y límites de reproducibilidad cross-platform.

### 3.2 Estructuras DOD exactas

Dibuja un mapa de memoria de las estructuras SoA; propone tipos (`float` o `double` según justificación), posiciones, velocidades, masas/inversas, flags, conectividad/constraints y buffers temporales específicos del dominio.

Incluye fragmentos C++20 de:
- `aligned_buffer` o almacenamiento con allocator explícito y liberación correcta.
- Vistas `std::span` no propietarias y separación entre capacidad y cantidad activa.
- Inicialización, crecimiento **fuera del hot loop**, reset y comprobación de capacidad.
- Lifetime y ownership de buffers, scratch por tarea y protección ante invalidaciones.
- Análisis de bytes por entidad y estimación de memoria total `M(N, ...)` en función de N y del dominio.

No permitas un `reinterpret_cast` indiscriminado ni supongas layout o alineación inexistente. Describe cómo reducir padding, fragmentación y false sharing.

### 3.3 Estructura espacial

Compara **Spatial Hash Grid, Uniform Grid compacta, Linear BVH y Octree lineal**, y escoge una solución viable para el MVP. Justifica complejidad temporal/memoria, localidad, coste de reconstrucción, coherencia temporal, distribución irregular, actualización paralela y tratamiento de overflow.

Incluye representación contigua (p. ej., cell keys, índices ordenados, offsets por celda), algoritmo de construcción, consultas y validación de vecinos. Explica cómo evitar asignaciones durante la simulación y cómo procesar límites del mundo/escenas vacías.

---

## 5. Entregable B — Diseño formal de la ABI y `include/dynquora/physics.h`

Redacta una **cabecera pública C11 autocontenida y representativa** que compile en C y C++, con:

1. Macro portable de exportación/importación, calling convention si corresponde y `#include <stdint.h>` / `<stddef.h>`.
2. API de **versionado** (`dq_get_api_version` o equivalente) y regla de compatibilidad semántica/ABI.
3. Tipos opacos `dq_context`, identificadores estables para entidades y reglas de invalidación.
4. `dq_result` con códigos claros (`OK`, `INVALID_ARGUMENT`, `OUT_OF_MEMORY`, `CAPACITY_EXCEEDED`, `INVALID_STATE`, `UNSUPPORTED_BACKEND`, `BUSY`, etc.).
5. `dq_allocator` con `allocate`, `deallocate` y `user_data`, especificación explícita de tamaño/alineación y simetría de liberación; evita wrappers ambiguos de `malloc/free` incapaces de garantizar alineación.
6. `dq_context_desc` versionado por `struct_size` y parámetros de capacidad, precisión, substeps, backend y configuración específica del dominio.
7. `dq_context_create(...)`, `dq_context_destroy(...)` y semántica del cierre con tareas pendientes.
8. Ingesta de datos: **copiar datos al almacenamiento propio** por defecto; si ofreces adopción o préstamo de buffers externos, diseña otra operación con ownership/lifetime inequívocos.
9. `dq_step(ctx, float dt)` o variante justificada, con comportamiento definido ante reentrada y threading.
10. Lecturas **zero-copy** mediante `const` pointer + count + stride + generation/epoch; definir exactamente cuánto duran las vistas, cuándo se invalidan y si pueden coexistir con escritura/step. Exponer una estrategia de pin/unpin o snapshots si hace falta.
11. Diagnósticos: obtención del último error o texto diagnóstico sin memoria compartida global insegura ni obligación de cruzar allocators.
12. Consulta de capacidades (`backend`, precisión, features, SIMD) sin prometer hardware inexistente.

Después de la cabecera, aporta **un ejemplo C compilable** de create → inject/load → step → acquire/read → release → destroy, con control de errores y sin acceso a `struct` opacos. Si no es viable incluir todas las implementaciones, declara con precisión que es una **interfaz propuesta**, no un enlace listo para ejecutar.

Añade una **tabla de contratos** de cada función: precondiciones, postcondiciones, complejidad, propiedad de memoria, thread safety, validación y errores.

### Bindings FFI

Explica los riesgos específicos para Rust (`unsafe`, lifetimes, `Send/Sync`), C# (P/Invoke, `SafeHandle`, pinning) y Python (`ctypes`/`cffi`, GIL, arrays NumPy), y elige **un primer binding** para la Fase 4. Define pruebas de integración y semántica de liberación.

---

## 6. Entregable C — Solver, paralelismo y ruta SIMD/GPU

### 5.1 Modelo físico y estabilidad

Especifica **ecuaciones, invariantes, algoritmo y orden de actualización** del dominio elegido. Documenta unidades físicas, parámetros de estabilidad, condiciones de contorno, tolerancias numéricas, singularidades, constraints/contactos y casos degenerados.

Proporciona al menos **dos pruebas matemáticas de referencia** con resultados esperados derivados, no datos inventados. Si usas aproximaciones numéricas, explica criterio de convergencia y error aceptable.

### 5.2 Job system neutral respecto al motor

Diseña una frontera mínima para ejecución secuencial o un scheduler suministrado por el host:

- `parallel_for` / dispatch y barrera/fence explícitos.
- Contexto de usuario, granularidad, chunking, scratch por worker y reglas de callbacks.
- Prohibición de callbacks reentrantes en puntos no seguros y mecanismo de propagación de errores.
- Scheduling sin necesidad de depender de hilos internos de Godot/Unreal; ofrece adaptadores fuera del núcleo.
- Evitar oversubscription, false sharing y escrituras concurrentes en buffers SoA.

Aporta pseudocódigo de dependencia por fases e identifica carreras potenciales y estrategias (double buffering, coloring, particiones, reducciones deterministas).

### 5.3 SIMD y backend

- Comparación de **escalar portable, auto-vectorización y kernels intrínsecos**.
- Alineación, tails, gather/scatter, SoA, aliasing, compilación multiversión y detección de capacidades en runtime.
- Estrategia de comparación bit a bit cuando corresponda o tolerancia numérica cuando no.
- Política de flags de coma flotante: `fast-math` **desactivado por defecto** salvo configuración de benchmark/compromiso explícito y documentado.
- Contrato interno de backend; CPU como referencia; GPU futuro con transferencias explícitas o staging. No asegurar zero-copy entre GPU y CPU si la plataforma no lo permite.
- Indicar cuándo una abstracción virtual o indirecta se resuelve **fuera** del hot loop.

---

## 7. Entregable D — Repositorio, CMake y empaquetado

Proporciona un árbol de proyecto realista que incluya como mínimo:

```text
DynquoraPhysics/
├── CMakeLists.txt
├── cmake/
├── include/dynquora/physics.h
├── src/core/
├── src/c_api/
├── src/backends/cpu/
├── src/spatial/
├── src/solvers/
├── src/schedulers/
├── tests/
├── benchmarks/
├── examples/c/
├── bindings/
├── docs/
├── LICENSE
├── NOTICE
├── README.md
├── CONTRIBUTING.md
├── CHANGELOG.md
└── SECURITY.md
```

Incluye un **`CMakeLists.txt` base realista y coherente con las rutas mostradas**:

- `cmake_minimum_required`, `project`, C++20 private, C ABI pública.
- Targets con nombres estables y `install()` / export de paquetes si es viable.
- Configuración `BUILD_SHARED_LIBS` o selección clara STATIC/SHARED; `POSITION_INDEPENDENT_CODE` cuando proceda.
- Opciones independientes para tests, benchmarks, sanitizers, warnings-as-errors, SIMD y `native` opcional.
- Flags correctos por compilador (GCC/Clang/MSVC), sin aplicar indiscriminadamente `-O3` en debug o `-march=native` en distribuciones.
- `-O3`/`/O2` en Release cuando proceda, LTO opcional y perfiles reproducibles de medición.
- Control de exportación de símbolos, instalación de cabeceras y configuración de include paths.
- Integración opt-in de `clang-tidy` / `cppcheck` si están instalados; evitar hacer fallar el build por herramientas opcionales ausentes.

Acompaña el script con **comandos de configuración, compilación, test e instalación** para Linux/macOS y Windows cuando difieran. Si solo construyes un esquema parcial, identifícalo y enumera archivos mínimos necesarios para completarlo.

### Documentación exigida

`README.md` debe explicar propósito, estado real, alcance, plataformas, requisitos, instrucciones de build, ejemplos de API C, modelo de memoria/ownership, integración host, límites numéricos, benchmarking y licencia. `NOTICE` debe respetar atribuciones reales, **sin inventar titulares de copyright**. `CHANGELOG.md` tendrá formato consistente y `SECURITY.md` un canal/procedimiento de reporte descrito como propuesta si aún no existe.

---

## 8. Entregable E — Rendimiento, validación y observabilidad

Define una estrategia de benchmark reproducible:

- Escenarios mínimos, medios y de estrés representativos del dominio; variación de N, densidad/topología, número de substeps y tamaño de dominio.
- Baseline escalar monohilo vs SoA optimizado vs SIMD vs multihilo, **manteniendo equivalencia funcional dentro de tolerancias**.
- Métricas: ms/step (mediana, p95/p99), steps/s, bytes/entity, RSS/pico de memoria, ciclos, instrucciones, cache misses L1/LLC, branch misses, ancho de banda estimado, escalado con número de threads.
- Instrumentación: `perf` (Linux), Cachegrind/Callgrind cuando aplique, herramientas de plataforma equivalentes, profiling de CPU y Tracy **solo opcional** si se justifica.
- Warm-up, pinning/afinidad opcional, CPU governor, builds Release reproducibles, hardware y compilador, repeticiones, intervalos de dispersión y exportación machine-readable (`csv`/`json`).
- Aclara que Cachegrind proporciona **simulación de eventos de caché**, no medidas directas de latencia real; diferencia contadores hardware de modelos.
- Presupuesto de rendimiento **parametrizable**, sin atribuir números reales hasta medir.

### Pruebas

Incluye matriz de pruebas unitarias, de propiedades/invariantes, regresión numérica, fuzzing, sanitizers, carga adversarial, estabilidad con `dt`, cross-platform C ABI y bindings. Define tolerancias para float/double y procedimientos para reproducir fallos.

Incluye una sección de **riesgos y mitigaciones**: divergencia SIMD, coste de sincronización, explosión de vecinos, crecimiento de capacidades, fragmentación, invalidación zero-copy, ABI breakage, calidad numérica y comportamiento en arquitecturas heterogéneas.

---

## 9. Entregable F — Roadmap MVP de cuatro fases, con gates claros

Para cada fase indica: objetivo, tareas en orden, interfaces y archivos afectados, dependencias previas, entregables verificables, pruebas, criterios de salida, riesgos, estimación relativa de esfuerzo (S/M/L o puntos, no fechas inventadas) y decisión de seguir/no seguir.

**Fase 1 — Referencia matemática y prototipo monohilo**
- Algoritmo correcto en CPU escalar, dataset determinista pequeño, tests de invariantes, límites y estabilidad; API interna simple.

**Fase 2 — DOD/SoA, allocations y SIMD**
- Migración a estructuras SoA, scratch preasignado, grid/BVH contiguo, profiling, vectorización y comparativa cuantitativa frente a la referencia.

**Fase 3 — Job system y API C**
- Scheduler agnóstico, fases sincronizadas, pruebas de carreras, `physics.h` estabilizada y ejemplos desde C.

**Fase 4 — Hardening, benchmark y binding**
- Stress tests, ASan/UBSan/TSan cuando corresponda, mediciones `perf`/Cachegrind, CI básica multi-OS, documentación y primer binding Rust o C#.

Propón un **backlog priorizado** de 10–20 tareas concretas para iniciar el desarrollo inmediatamente; cada tarea debe tener resultado observable y un criterio de aceptación.

---

## 10. Integración con Godot, Unreal y motores propios

Describe **adaptadores finos** separados del núcleo para:

- **Godot:** frontera GDExtension o enlace nativo, ciclo de actualización, sincronización de estado y condiciones de despliegue.
- **Unreal:** módulo/plugin puente, tick/substepping, threading y conversión de datos sin asumir acceso a internals privados.
- **In-house:** uso directo de C ABI, integración con job system de anfitrión, render buffers y observabilidad.

Analiza transformaciones de coordenadas, unidades, convenciones de matrices, lifetime, diferencias entre thread de simulación y render, costes de copia y condiciones de lectura segura. Evita afirmar integración lista para producción sin construir y probar adaptadores.

---

## 11. Formato de la respuesta (orden obligatorio)

Entrega **una especificación técnica extensa pero organizada**, con este orden:

1. **Executive summary** (máximo 250 palabras) y selección de dominio.
2. **Supuestos, alcance, exclusiones y decisiones pendientes**.
3. **ADR resumidos**: tabla de decisiones, alternativas rechazadas y justificación.
4. **Arquitectura por capas** y diagrama Mermaid del pipeline.
5. **Diseño de memoria SoA y estructuras espaciales** con código C++20 y presupuesto de memoria.
6. **Solver**: fundamentos matemáticos, pseudocódigo, invariantes, estabilidad y tests de referencia.
7. **API C**: cabecera `physics.h` y ejemplo consumidor en C; tabla de contratos.
8. **Multihilo, SIMD y evolución GPU**.
9. **Integraciones** Godot, Unreal e in-house.
10. **Árbol de repositorio, `CMakeLists.txt` y comandos de build**.
11. **Plan de pruebas, seguridad, métricas y benchmarking**.
12. **Roadmap de cuatro fases**, gates y backlog priorizado.
13. **Matriz de trazabilidad** requisito → componente → prueba → fase.
14. **Riesgos, trade-offs, bloqueadores y preguntas abiertas**, incluida la validación final de la marca Dynquora.
15. **Lista de verificación final** para validar la propuesta.

Cuando aportes código:
- Indica ruta destino sobre el bloque (`include/...`, `src/...`, etc.).
- Utiliza bloques de lenguaje `c`, `cpp`, `cmake`, `bash` o `mermaid`.
- Mantén consistencia de nombres y firma entre header y ejemplos.
- No omitas los includes esenciales ni presupongas una configuración mágica.
- Identifica qué es **especificación**, qué es **fragmento demostrativo** y qué es **implementación completa**.

---

## 12. Lista de verificación de calidad (autoevaluación antes de responder)

Comprueba que:

- [ ] El proyecto utiliza Dynquora de manera coherente y no supone un registro de marca confirmado.
- [ ] Se eligió un único dominio y todo el diseño es específico de él.
- [ ] El algoritmo físico y el orden del pipeline son coherentes con las ecuaciones.
- [ ] El layout SoA tiene capacidad, ownership, alineación y presupuesto de memoria explícitos.
- [ ] El hot loop no requiere allocaciones; los casos de capacidad agotada están definidos.
- [ ] La cabecera es válida para C11, no expone C++ ni excepciones y maneja errores.
- [ ] Zero-copy tiene una política inequívoca de generación, lifetime, locking e invalidación.
- [ ] La API describe thread safety, destrucción y reentrancia.
- [ ] Hay fallback escalar y `-march=native`/`fast-math` no son predeterminados para distribución.
- [ ] Paralelismo evita carreras de lectura-escritura y tiene barreras definidas.
- [ ] La configuración CMake distingue plataforma, compilador y Debug/Release.
- [ ] Los ejemplos C/C++ usan nombres y tipos coherentes entre sí.
- [ ] Existen pruebas matemáticas, sanitizers, benchmarking y medición reproducible.
- [ ] Los benchmarks que no se han ejecutado se etiquetan como **pendientes de medición**.
- [ ] El roadmap contiene cuatro fases, criterios de salida medibles y backlog accionable.
- [ ] El diseño declara límites, costes y riesgos sin promesas infundadas.

**Identidad visual:** si se emplea el símbolo de Dynquora en README, documentación, ejemplos o adaptadores, tratarlo como recurso gráfico separado de la API y no hacer que el núcleo dependa de assets de branding. Incluir en la propuesta una recomendación de carpeta `assets/branding/` para el icono y manual mínimo de uso.

**Instrucción final:** entrega una propuesta que otro equipo senior pueda revisar, implementar de forma incremental, probar y mantener durante años. Prioriza especificidad, integridad y validez técnica sobre volumen superficial.
