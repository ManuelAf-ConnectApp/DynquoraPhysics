#pragma once

#include <dynquora/physics.h>

#include <cstdint>
#include <string>
#include <vector>

struct Constraint {
  uint32_t a;
  uint32_t b;
  float rest;
  float compliance;
  float lambda;
};

struct GridEntry {
  int32_t x;
  int32_t y;
  int32_t z;
  uint32_t particle;
};

struct ParticleContact {
  uint32_t a;
  uint32_t b;
  float lambda;
  float nx;
  float ny;
  float nz;
  float incoming_normal_speed;
  uint64_t last_seen_step;
};

struct Particles {
  std::vector<float> px, py, pz, prev_x, prev_y, prev_z, vx, vy, vz, inverse_mass;
  std::vector<dq_vec3> published_positions, published_velocities;

  void reserve(uint32_t capacity);
  [[nodiscard]] size_t size() const;
  void add(const dq_particle_desc& particle);
};

struct dq_context {
  uint32_t max_particles, max_constraints, iterations, substeps;
  dq_vec3 gravity;
  float linear_damping;
  float grid_cell_size, floor_height, floor_restitution, floor_friction;
  float particle_radius, particle_contact_compliance, particle_restitution, particle_friction;
  bool floor_enabled;
  Particles particles;
  std::vector<Constraint> constraints;
  std::vector<GridEntry> grid_entries;
  std::vector<ParticleContact> particle_contacts;
  dq_stats stats{};
  uint64_t generation = 0;
  bool view_held = false;
  std::string error;
};
