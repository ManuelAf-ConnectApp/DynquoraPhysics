#ifndef DYNQUORA_PHYSICS_H
#define DYNQUORA_PHYSICS_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(DQ_BUILD_SHARED)
#    if defined(DQ_BUILDING_LIBRARY)
#      define DQ_API __declspec(dllexport)
#    else
#      define DQ_API __declspec(dllimport)
#    endif
#  else
#    define DQ_API
#  endif
#  define DQ_CALL __cdecl
#else
#  define DQ_API __attribute__((visibility("default")))
#  define DQ_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DQ_API_VERSION_MAJOR 0u
#define DQ_API_VERSION_MINOR 5u
#define DQ_API_VERSION_PATCH 0u

typedef struct dq_context dq_context;
typedef uint32_t dq_particle_id;
typedef uint32_t dq_constraint_id;

typedef enum dq_result {
  DQ_OK = 0,
  DQ_INVALID_ARGUMENT,
  DQ_OUT_OF_MEMORY,
  DQ_CAPACITY_EXCEEDED,
  DQ_INVALID_STATE,
  DQ_UNSUPPORTED_BACKEND,
  DQ_BUSY,
  DQ_NUMERICAL_ERROR,
  DQ_INTERNAL_ERROR
} dq_result;

typedef enum dq_backend { DQ_BACKEND_CPU = 0 } dq_backend;

typedef struct dq_vec3 { float x, y, z; } dq_vec3;

typedef void* (DQ_CALL *dq_allocate_fn)(void* user_data, size_t size, size_t alignment);
typedef void (DQ_CALL *dq_deallocate_fn)(void* user_data, void* memory, size_t size, size_t alignment);
typedef struct dq_allocator {
  dq_allocate_fn allocate;
  dq_deallocate_fn deallocate;
  void* user_data;
} dq_allocator;

typedef struct dq_context_desc {
  uint32_t struct_size;
  uint32_t max_particles;
  uint32_t max_distance_constraints;
  uint32_t solver_iterations;
  uint32_t substeps;
  dq_vec3 gravity;
  float linear_damping; /* [0, 1], applied once per substep to dynamic velocities. */
  float grid_cell_size; /* Positive enables the preallocated 3D compact uniform grid. */
  uint32_t enable_floor;
  float floor_height;
  float floor_restitution; /* [0, 1] */
  float floor_friction; /* [0, 1] */
  float particle_radius; /* Positive enables contacts; grid_cell_size must be >= 2 * radius. */
  uint32_t max_particle_contacts; /* Zero selects max_particles; capacity is never grown in dq_step. */
  float particle_contact_compliance; /* Non-negative XPBD compliance. */
  float particle_restitution; /* [0, 1], applied to approaching particle contacts. */
  float particle_friction; /* [0, 1], tangential velocity damping at contacts. */
  dq_backend backend;
  dq_allocator allocator; /* Both callbacks must be set, or both must be NULL. */
} dq_context_desc;

typedef struct dq_particle_desc {
  dq_vec3 position;
  dq_vec3 velocity;
  float inverse_mass; /* zero denotes a kinematic/pinned particle. */
} dq_particle_desc;

typedef struct dq_distance_constraint_desc {
  dq_particle_id first;
  dq_particle_id second;
  float rest_length;
  float compliance; /* Non-negative XPBD compliance, in m/N. */
} dq_distance_constraint_desc;

typedef struct dq_particle_view {
  const dq_vec3* positions;
  const dq_vec3* velocities;
  uint32_t count;
  uint64_t generation;
} dq_particle_view;

typedef struct dq_stats {
  uint64_t step_count;
  uint64_t last_step_nanoseconds;
  uint32_t particle_count;
  uint32_t distance_constraint_count;
  uint32_t grid_entry_count;
  uint32_t contact_pair_count;
  uint32_t cached_contact_count;
  uint32_t dropped_contact_count;
  float maximum_penetration;
} dq_stats;

DQ_API uint32_t DQ_CALL dq_get_api_version(void);
DQ_API const char* DQ_CALL dq_result_string(dq_result result);
DQ_API dq_result DQ_CALL dq_context_create(const dq_context_desc* desc, dq_context** out_context);
DQ_API void DQ_CALL dq_context_destroy(dq_context* context);
DQ_API dq_result DQ_CALL dq_context_add_particle(dq_context* context, const dq_particle_desc* desc, dq_particle_id* out_id);
DQ_API dq_result DQ_CALL dq_context_add_distance_constraint(dq_context* context, const dq_distance_constraint_desc* desc, dq_constraint_id* out_id);
DQ_API dq_result DQ_CALL dq_step(dq_context* context, float dt);
/* A view is valid until dq_particles_release or context destruction. Stepping while held returns DQ_BUSY. */
DQ_API dq_result DQ_CALL dq_particles_acquire(dq_context* context, dq_particle_view* out_view);
DQ_API dq_result DQ_CALL dq_particles_release(dq_context* context, uint64_t generation);
DQ_API dq_result DQ_CALL dq_context_last_error(const dq_context* context, char* buffer, size_t buffer_size, size_t* out_required_size);
DQ_API dq_result DQ_CALL dq_context_get_stats(const dq_context* context, dq_stats* out_stats);

#ifdef __cplusplus
}
#endif
#endif
