#include <dynquora/physics.h>
#include <stdio.h>

int main(void) {
  dq_context_desc desc = {0};
  desc.struct_size = sizeof desc; desc.max_particles = 2; desc.max_distance_constraints = 1;
  desc.solver_iterations = 8; desc.substeps = 1; desc.gravity = (dq_vec3){0.0f, -9.81f, 0.0f};
  dq_context* ctx = NULL;
  if (dq_context_create(&desc, &ctx) != DQ_OK) return 1;
  dq_particle_id a, b; dq_constraint_id link;
  dq_particle_desc anchor = {{0, 0, 0}, {0, 0, 0}, 0.0f};
  dq_particle_desc mass = {{1, 0, 0}, {0, 0, 0}, 1.0f};
  if (dq_context_add_particle(ctx, &anchor, &a) || dq_context_add_particle(ctx, &mass, &b)) return 2;
  dq_distance_constraint_desc constraint = {a, b, 1.0f, 0.0f};
  if (dq_context_add_distance_constraint(ctx, &constraint, &link) || dq_step(ctx, 1.0f / 60.0f)) return 3;
  dq_particle_view view;
  if (dq_particles_acquire(ctx, &view) == DQ_OK) { printf("particle 1: %f %f %f\n", view.positions[1].x, view.positions[1].y, view.positions[1].z); dq_particles_release(ctx, view.generation); }
  dq_context_destroy(ctx); return 0;
}
