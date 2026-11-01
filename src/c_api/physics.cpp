#include <dynquora/physics.h>

#include "core/context.hpp"
#include "contacts/particle_contacts.hpp"
#include "solvers/xpbd.hpp"
#include "spatial/uniform_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <new>

void Particles::reserve(uint32_t capacity) {
  px.reserve(capacity); py.reserve(capacity); pz.reserve(capacity);
  prev_x.reserve(capacity); prev_y.reserve(capacity); prev_z.reserve(capacity);
  vx.reserve(capacity); vy.reserve(capacity); vz.reserve(capacity); inverse_mass.reserve(capacity);
  published_positions.reserve(capacity); published_velocities.reserve(capacity);
}
size_t Particles::size() const { return px.size(); }
void Particles::add(const dq_particle_desc& particle) {
  px.push_back(particle.position.x); py.push_back(particle.position.y); pz.push_back(particle.position.z);
  prev_x.push_back(particle.position.x); prev_y.push_back(particle.position.y); prev_z.push_back(particle.position.z);
  vx.push_back(particle.velocity.x); vy.push_back(particle.velocity.y); vz.push_back(particle.velocity.z);
  inverse_mass.push_back(particle.inverse_mass);
}

namespace {
bool finite_vec(dq_vec3 value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }
void set_error(dq_context* context, const char* message) noexcept {
  if (!context) return;
  try { context->error = message; } catch (...) { }
}
bool valid_desc(const dq_context_desc& desc) {
  const bool contacts_are_configured = desc.particle_radius == 0.0f || desc.grid_cell_size >= 2.0f * desc.particle_radius;
  return desc.max_particles > 0 && desc.solver_iterations > 0 && desc.substeps > 0 && desc.backend == DQ_BACKEND_CPU &&
    finite_vec(desc.gravity) && std::isfinite(desc.linear_damping) && desc.linear_damping >= 0.0f && desc.linear_damping <= 1.0f &&
    std::isfinite(desc.grid_cell_size) && desc.grid_cell_size >= 0.0f && desc.enable_floor <= 1 &&
    std::isfinite(desc.particle_radius) && desc.particle_radius >= 0.0f && contacts_are_configured &&
    std::isfinite(desc.particle_contact_compliance) && desc.particle_contact_compliance >= 0.0f &&
    std::isfinite(desc.particle_restitution) && desc.particle_restitution >= 0.0f && desc.particle_restitution <= 1.0f &&
    std::isfinite(desc.particle_friction) && desc.particle_friction >= 0.0f && desc.particle_friction <= 1.0f &&
    (!desc.enable_floor || (std::isfinite(desc.floor_height) && std::isfinite(desc.floor_restitution) && std::isfinite(desc.floor_friction) &&
      desc.floor_restitution >= 0.0f && desc.floor_restitution <= 1.0f && desc.floor_friction >= 0.0f && desc.floor_friction <= 1.0f)) &&
    ((desc.allocator.allocate == nullptr && desc.allocator.deallocate == nullptr) || (desc.allocator.allocate != nullptr && desc.allocator.deallocate != nullptr));
}
void integrate(Particles& particles, dq_vec3 gravity, float damping, float step_size) {
  for (size_t index = 0; index < particles.size(); ++index) {
    particles.prev_x[index] = particles.px[index]; particles.prev_y[index] = particles.py[index]; particles.prev_z[index] = particles.pz[index];
    if (particles.inverse_mass[index] <= 0.0f) continue;
    particles.vx[index] = (particles.vx[index] + gravity.x * step_size) * (1.0f - damping);
    particles.vy[index] = (particles.vy[index] + gravity.y * step_size) * (1.0f - damping);
    particles.vz[index] = (particles.vz[index] + gravity.z * step_size) * (1.0f - damping);
    particles.px[index] += particles.vx[index] * step_size; particles.py[index] += particles.vy[index] * step_size; particles.pz[index] += particles.vz[index] * step_size;
  }
}
void derive_velocities(Particles& particles, float step_size) {
  for (size_t index = 0; index < particles.size(); ++index) if (particles.inverse_mass[index] > 0.0f) {
    particles.vx[index] = (particles.px[index] - particles.prev_x[index]) / step_size;
    particles.vy[index] = (particles.py[index] - particles.prev_y[index]) / step_size;
    particles.vz[index] = (particles.pz[index] - particles.prev_z[index]) / step_size;
  }
}
void solve_floor(Particles& particles, const dq_context& context) {
  if (!context.floor_enabled) return;
  for (size_t index = 0; index < particles.size(); ++index) if (particles.inverse_mass[index] > 0.0f && particles.py[index] < context.floor_height) {
    particles.py[index] = context.floor_height;
    if (particles.vy[index] < 0.0f) particles.vy[index] = -particles.vy[index] * context.floor_restitution;
    particles.vx[index] *= 1.0f - context.floor_friction; particles.vz[index] *= 1.0f - context.floor_friction;
  }
}
dq_result step(dq_context& context, float dt) {
  const float step_size = dt / static_cast<float>(context.substeps);
  if (!std::isfinite(step_size) || step_size <= 0.0f || step_size * step_size == 0.0f) return DQ_INVALID_ARGUMENT;
  const auto started_at = std::chrono::steady_clock::now();
  context.particle_contacts.erase(std::remove_if(context.particle_contacts.begin(), context.particle_contacts.end(), [&context](const ParticleContact& contact) { return contact.last_seen_step + 4 < context.stats.step_count; }), context.particle_contacts.end());
  context.stats.contact_pair_count = 0; context.stats.dropped_contact_count = 0; context.stats.maximum_penetration = 0.0f;
  for (uint32_t substep = 0; substep < context.substeps; ++substep) {
    integrate(context.particles, context.gravity, context.linear_damping, step_size);
    rebuild_uniform_grid(context.particles, context.grid_cell_size, context.grid_entries);
    solve_distance_constraints(context.particles, context.constraints, context.iterations, step_size);
    solve_particle_contacts(context, step_size); derive_velocities(context.particles, step_size); resolve_contact_velocities(context); solve_floor(context.particles, context);
  }
  for (size_t index = 0; index < context.particles.size(); ++index) if (!std::isfinite(context.particles.px[index]) || !std::isfinite(context.particles.py[index]) || !std::isfinite(context.particles.pz[index])) { set_error(&context, "non-finite position produced by solver"); return DQ_NUMERICAL_ERROR; }
  ++context.generation; ++context.stats.step_count;
  context.stats.last_step_nanoseconds = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - started_at).count());
  context.stats.particle_count = static_cast<uint32_t>(context.particles.size()); context.stats.distance_constraint_count = static_cast<uint32_t>(context.constraints.size()); context.stats.grid_entry_count = static_cast<uint32_t>(context.grid_entries.size()); context.stats.cached_contact_count = static_cast<uint32_t>(context.particle_contacts.size());
  return DQ_OK;
}
} // namespace

extern "C" {
uint32_t DQ_CALL dq_get_api_version(void) { return (DQ_API_VERSION_MAJOR << 16u) | (DQ_API_VERSION_MINOR << 8u) | DQ_API_VERSION_PATCH; }
const char* DQ_CALL dq_result_string(dq_result result) { static const char* const text[] = {"OK", "invalid argument", "out of memory", "capacity exceeded", "invalid state", "unsupported backend", "busy", "numerical error", "internal error"}; return (result >= DQ_OK && result <= DQ_INTERNAL_ERROR) ? text[result] : "unknown result"; }
dq_result DQ_CALL dq_context_create(const dq_context_desc* desc, dq_context** out_context) {
  if (!out_context) return DQ_INVALID_ARGUMENT;
  *out_context = nullptr;
  if (!desc || desc->struct_size < sizeof(dq_context_desc) || !valid_desc(*desc)) return DQ_INVALID_ARGUMENT;
  if (desc->allocator.allocate != nullptr) return DQ_UNSUPPORTED_BACKEND;
  try {
    auto context = std::make_unique<dq_context>();
    context->max_particles = desc->max_particles; context->max_constraints = desc->max_distance_constraints; context->iterations = desc->solver_iterations; context->substeps = desc->substeps; context->gravity = desc->gravity; context->linear_damping = desc->linear_damping; context->grid_cell_size = desc->grid_cell_size; context->floor_enabled = desc->enable_floor != 0; context->floor_height = desc->floor_height; context->floor_restitution = desc->floor_restitution; context->floor_friction = desc->floor_friction; context->particle_radius = desc->particle_radius; context->particle_contact_compliance = desc->particle_contact_compliance; context->particle_restitution = desc->particle_restitution; context->particle_friction = desc->particle_friction;
    context->particles.reserve(desc->max_particles); context->constraints.reserve(desc->max_distance_constraints); context->grid_entries.reserve(desc->max_particles); context->particle_contacts.reserve(desc->max_particle_contacts ? desc->max_particle_contacts : desc->max_particles);
    *out_context = context.release(); return DQ_OK;
  } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { return DQ_INTERNAL_ERROR; }
}
void DQ_CALL dq_context_destroy(dq_context* context) { delete context; }
dq_result DQ_CALL dq_context_add_particle(dq_context* context, const dq_particle_desc* desc, dq_particle_id* out_id) {
  if (!context || !desc || !out_id || !finite_vec(desc->position) || !finite_vec(desc->velocity) || !std::isfinite(desc->inverse_mass) || desc->inverse_mass < 0.0f) return DQ_INVALID_ARGUMENT;
  if (context->view_held) return DQ_BUSY;
  if (context->particles.size() == context->max_particles) { set_error(context, "particle capacity exhausted"); return DQ_CAPACITY_EXCEEDED; }
  try { *out_id = static_cast<uint32_t>(context->particles.size()); context->particles.add(*desc); return DQ_OK; } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { return DQ_INTERNAL_ERROR; }
}
dq_result DQ_CALL dq_context_add_distance_constraint(dq_context* context, const dq_distance_constraint_desc* desc, dq_constraint_id* out_id) {
  if (!context || !desc || !out_id || desc->first >= context->particles.size() || desc->second >= context->particles.size() || desc->first == desc->second || !std::isfinite(desc->rest_length) || !std::isfinite(desc->compliance) || desc->rest_length < 0.0f || desc->compliance < 0.0f) return DQ_INVALID_ARGUMENT;
  if (context->view_held) return DQ_BUSY;
  if (context->constraints.size() == context->max_constraints) return DQ_CAPACITY_EXCEEDED;
  try { *out_id = static_cast<uint32_t>(context->constraints.size()); context->constraints.push_back({desc->first, desc->second, desc->rest_length, desc->compliance, 0.0f}); return DQ_OK; } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { return DQ_INTERNAL_ERROR; }
}
dq_result DQ_CALL dq_step(dq_context* context, float dt) {
  if (!context || !std::isfinite(dt) || dt <= 0.0f) return DQ_INVALID_ARGUMENT;
  if (context->view_held) return DQ_BUSY;
  try { return step(*context, dt); } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { set_error(context, "internal solver error"); return DQ_INTERNAL_ERROR; }
}
dq_result DQ_CALL dq_particles_acquire(dq_context* context, dq_particle_view* out_view) {
  if (!context || !out_view) return DQ_INVALID_ARGUMENT;
  if (context->view_held) return DQ_BUSY;
  try {
    Particles& particles = context->particles; particles.published_positions.resize(particles.size()); particles.published_velocities.resize(particles.size());
    for (size_t index = 0; index < particles.size(); ++index) { particles.published_positions[index] = {particles.px[index], particles.py[index], particles.pz[index]}; particles.published_velocities[index] = {particles.vx[index], particles.vy[index], particles.vz[index]}; }
    context->view_held = true; *out_view = {particles.published_positions.data(), particles.published_velocities.data(), static_cast<uint32_t>(particles.size()), context->generation}; return DQ_OK;
  } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { return DQ_INTERNAL_ERROR; }
}
dq_result DQ_CALL dq_particles_release(dq_context* context, uint64_t generation) { if (!context || !context->view_held || generation != context->generation) return DQ_INVALID_STATE; context->view_held = false; return DQ_OK; }
dq_result DQ_CALL dq_context_last_error(const dq_context* context, char* buffer, size_t buffer_size, size_t* out_required_size) { if (!context) return DQ_INVALID_ARGUMENT; const size_t required_size = context->error.size() + 1; if (out_required_size) *out_required_size = required_size; if (!buffer) return buffer_size == 0 ? DQ_OK : DQ_INVALID_ARGUMENT; if (buffer_size < required_size) return DQ_CAPACITY_EXCEEDED; std::memcpy(buffer, context->error.c_str(), required_size); return DQ_OK; }
dq_result DQ_CALL dq_context_get_stats(const dq_context* context, dq_stats* out_stats) { if (!context || !out_stats) return DQ_INVALID_ARGUMENT; *out_stats = context->stats; return DQ_OK; }
}
