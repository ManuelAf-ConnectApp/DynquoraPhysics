#include "contacts/particle_contacts.hpp"

#include "spatial/uniform_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
int32_t neighbouring_cell_coordinate(int32_t value, int offset) {
  if (offset < 0 && value == std::numeric_limits<int32_t>::min()) return value;
  if (offset > 0 && value == std::numeric_limits<int32_t>::max()) return value;
  return static_cast<int32_t>(value + offset);
}

auto cell_begin(std::vector<GridEntry>& entries, int32_t x, int32_t y, int32_t z) {
  return std::lower_bound(entries.begin(), entries.end(), GridEntry{x, y, z, 0}, [](const GridEntry& left, const GridEntry& right) {
    if (left.x != right.x) return left.x < right.x;
    if (left.y != right.y) return left.y < right.y;
    return left.z < right.z;
  });
}
}

void solve_particle_contacts(dq_context& context, float step_size) {
  if (context.particle_radius <= 0.0f || context.grid_cell_size <= 0.0f) return;
  Particles& particles = context.particles;
  const float minimum_distance = 2.0f * context.particle_radius;
  const float alpha = context.particle_contact_compliance / (step_size * step_size);
  for (const GridEntry& entry : context.grid_entries) for (int z_offset = -1; z_offset <= 1; ++z_offset) for (int y_offset = -1; y_offset <= 1; ++y_offset) for (int x_offset = -1; x_offset <= 1; ++x_offset) {
    const int32_t x = neighbouring_cell_coordinate(entry.x, x_offset), y = neighbouring_cell_coordinate(entry.y, y_offset), z = neighbouring_cell_coordinate(entry.z, z_offset);
    for (auto candidate = cell_begin(context.grid_entries, x, y, z); candidate != context.grid_entries.end() && same_cell(*candidate, x, y, z); ++candidate) {
      const uint32_t a = entry.particle, b = candidate->particle;
      if (b <= a) continue;
      float dx = particles.px[a] - particles.px[b], dy = particles.py[a] - particles.py[b], dz = particles.pz[a] - particles.pz[b];
      const float distance_squared = dx * dx + dy * dy + dz * dz;
      if (distance_squared >= minimum_distance * minimum_distance) continue;
      ++context.stats.contact_pair_count;
      float distance = std::sqrt(distance_squared);
      if (distance <= std::numeric_limits<float>::epsilon()) { distance = 1.0e-6f; dx = (a & 1u) ? distance : -distance; dy = dz = 0.0f; }
      context.stats.maximum_penetration = std::max(context.stats.maximum_penetration, minimum_distance - distance);
      const float inverse_mass_sum = particles.inverse_mass[a] + particles.inverse_mass[b];
      if (inverse_mass_sum <= 0.0f) continue;
      ParticleContact* contact = nullptr;
      for (ParticleContact& cached : context.particle_contacts) if (cached.a == a && cached.b == b) { contact = &cached; break; }
      if (!contact) {
        if (context.particle_contacts.size() == context.particle_contacts.capacity()) { ++context.stats.dropped_contact_count; continue; }
        context.particle_contacts.push_back({a, b, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0}); contact = &context.particle_contacts.back();
      }
      contact->last_seen_step = context.stats.step_count + 1; contact->nx = dx / distance; contact->ny = dy / distance; contact->nz = dz / distance;
      const float rvx = particles.vx[a] - particles.vx[b], rvy = particles.vy[a] - particles.vy[b], rvz = particles.vz[a] - particles.vz[b];
      contact->incoming_normal_speed = rvx * contact->nx + rvy * contact->ny + rvz * contact->nz;
      const float old_lambda = contact->lambda;
      contact->lambda = std::max(0.0f, old_lambda + (minimum_distance - distance - alpha * old_lambda) / (inverse_mass_sum + alpha));
      const float correction = (contact->lambda - old_lambda) / distance;
      particles.px[a] += dx * correction * particles.inverse_mass[a]; particles.py[a] += dy * correction * particles.inverse_mass[a]; particles.pz[a] += dz * correction * particles.inverse_mass[a];
      particles.px[b] -= dx * correction * particles.inverse_mass[b]; particles.py[b] -= dy * correction * particles.inverse_mass[b]; particles.pz[b] -= dz * correction * particles.inverse_mass[b];
    }
  }
}

void resolve_contact_velocities(dq_context& context) {
  Particles& particles = context.particles;
  for (const ParticleContact& contact : context.particle_contacts) if (contact.last_seen_step == context.stats.step_count + 1) {
    const float inverse_mass_sum = particles.inverse_mass[contact.a] + particles.inverse_mass[contact.b];
    if (inverse_mass_sum <= 0.0f) continue;
    float rvx = particles.vx[contact.a] - particles.vx[contact.b], rvy = particles.vy[contact.a] - particles.vy[contact.b], rvz = particles.vz[contact.a] - particles.vz[contact.b];
    const float normal_speed = rvx * contact.nx + rvy * contact.ny + rvz * contact.nz;
    const float target_speed = contact.incoming_normal_speed < 0.0f ? -contact.incoming_normal_speed * context.particle_restitution : 0.0f;
    if (normal_speed < target_speed) {
      const float impulse = (target_speed - normal_speed) / inverse_mass_sum;
      particles.vx[contact.a] += contact.nx * impulse * particles.inverse_mass[contact.a]; particles.vy[contact.a] += contact.ny * impulse * particles.inverse_mass[contact.a]; particles.vz[contact.a] += contact.nz * impulse * particles.inverse_mass[contact.a];
      particles.vx[contact.b] -= contact.nx * impulse * particles.inverse_mass[contact.b]; particles.vy[contact.b] -= contact.ny * impulse * particles.inverse_mass[contact.b]; particles.vz[contact.b] -= contact.nz * impulse * particles.inverse_mass[contact.b];
    }
    const float tangent_x = rvx - normal_speed * contact.nx, tangent_y = rvy - normal_speed * contact.ny, tangent_z = rvz - normal_speed * contact.nz;
    particles.vx[contact.a] -= tangent_x * context.particle_friction * particles.inverse_mass[contact.a] / inverse_mass_sum; particles.vy[contact.a] -= tangent_y * context.particle_friction * particles.inverse_mass[contact.a] / inverse_mass_sum; particles.vz[contact.a] -= tangent_z * context.particle_friction * particles.inverse_mass[contact.a] / inverse_mass_sum;
    particles.vx[contact.b] += tangent_x * context.particle_friction * particles.inverse_mass[contact.b] / inverse_mass_sum; particles.vy[contact.b] += tangent_y * context.particle_friction * particles.inverse_mass[contact.b] / inverse_mass_sum; particles.vz[contact.b] += tangent_z * context.particle_friction * particles.inverse_mass[contact.b] / inverse_mass_sum;
  }
}
