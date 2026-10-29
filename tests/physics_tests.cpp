#include <dynquora/physics.h>
#include <cassert>
#include <cmath>

int main() {
  dq_context_desc d{}; d.struct_size=sizeof d; d.max_particles=2; d.max_distance_constraints=1; d.solver_iterations=16; d.substeps=1; d.gravity={0,0,0};
  dq_context* c=nullptr; assert(dq_context_create(&d,&c)==DQ_OK);
  dq_particle_id a,b; dq_constraint_id link; dq_particle_desc p0{{0,0,0},{0,0,0},0}, p1{{2,0,0},{0,0,0},1};
  assert(dq_context_add_particle(c,&p0,&a)==DQ_OK && dq_context_add_particle(c,&p1,&b)==DQ_OK);
  dq_distance_constraint_desc q{a,b,1.0f,0}; assert(dq_context_add_distance_constraint(c,&q,&link)==DQ_OK);
  assert(dq_step(c,1.0f/60.0f)==DQ_OK); dq_particle_view v{}; assert(dq_particles_acquire(c,&v)==DQ_OK);
  assert(dq_step(c,1.0f/60.0f)==DQ_BUSY);
  float dx=v.positions[b].x-v.positions[a].x; assert(std::fabs(dx-1.0f)<1e-4f); assert(dq_particles_release(c,v.generation)==DQ_OK);
  assert(dq_step(c,0)==DQ_INVALID_ARGUMENT); dq_context_destroy(c);
}

static void test_conservation_capacity_and_validation() {
  dq_context_desc d{}; d.struct_size=sizeof d; d.max_particles=1; d.max_distance_constraints=0; d.solver_iterations=1; d.substeps=1;
  dq_context* c=nullptr; assert(dq_context_create(&d,&c)==DQ_OK);
  dq_particle_id id; dq_particle_desc p{{1,2,3},{2,-3,4},1};
  assert(dq_context_add_particle(c,&p,&id)==DQ_OK);
  assert(dq_step(c,0.25f)==DQ_OK); dq_particle_view v{}; assert(dq_particles_acquire(c,&v)==DQ_OK);
  assert(std::fabs(v.positions[id].x-1.5f)<1e-6f && std::fabs(v.positions[id].y-1.25f)<1e-6f && std::fabs(v.velocities[id].z-4.0f)<1e-6f);
  assert(dq_particles_release(c,v.generation)==DQ_OK);
  assert(dq_context_add_particle(c,&p,&id)==DQ_CAPACITY_EXCEEDED);
  p.position.x=INFINITY; assert(dq_context_add_particle(c,&p,&id)==DQ_INVALID_ARGUMENT);
  dq_context_destroy(c);
  d.linear_damping=2.0f; assert(dq_context_create(&d,&c)==DQ_INVALID_ARGUMENT);
}

static void test_floor_collision() {
  dq_context_desc d{}; d.struct_size=sizeof d; d.max_particles=1; d.solver_iterations=1; d.substeps=1; d.gravity={0,-10,0}; d.grid_cell_size=0.5f; d.enable_floor=1; d.floor_height=0; d.floor_restitution=0.5f; d.floor_friction=0;
  dq_context* c=nullptr; assert(dq_context_create(&d,&c)==DQ_OK);
  dq_particle_id id; dq_particle_desc p{{0,0.1f,0},{0,-2,0},1}; assert(dq_context_add_particle(c,&p,&id)==DQ_OK);
  assert(dq_step(c,0.1f)==DQ_OK); dq_particle_view v{}; assert(dq_particles_acquire(c,&v)==DQ_OK);
  assert(v.positions[id].y >= 0.0f && v.velocities[id].y > 0.0f);
  assert(dq_particles_release(c,v.generation)==DQ_OK); dq_context_destroy(c);
}

static void test_particle_contact() {
  dq_context_desc d{}; d.struct_size=sizeof d; d.max_particles=2; d.solver_iterations=4; d.substeps=1; d.grid_cell_size=1; d.particle_radius=0.5f; d.max_particle_contacts=1;
  dq_context* c=nullptr; assert(dq_context_create(&d,&c)==DQ_OK);
  dq_particle_id a,b; dq_particle_desc p{{0,0,0},{0,0,0},1}; assert(dq_context_add_particle(c,&p,&a)==DQ_OK); p.position.x=0.25f; assert(dq_context_add_particle(c,&p,&b)==DQ_OK);
  assert(dq_step(c,0.1f)==DQ_OK); dq_particle_view v{}; assert(dq_particles_acquire(c,&v)==DQ_OK);
  assert(std::fabs(v.positions[a].x-v.positions[b].x) >= 0.999f);
  assert(dq_particles_release(c,v.generation)==DQ_OK); dq_stats stats{}; assert(dq_context_get_stats(c,&stats)==DQ_OK && stats.cached_contact_count==1); dq_context_destroy(c);
}

static void test_stats() {
  dq_context_desc d{}; d.struct_size=sizeof d; d.max_particles=1; d.solver_iterations=1; d.substeps=1;
  dq_context* c = nullptr; assert(dq_context_create(&d,&c)==DQ_OK);
  dq_particle_id id; dq_particle_desc p{{0,0,0},{0,0,0},1}; assert(dq_context_add_particle(c,&p,&id)==DQ_OK); assert(dq_step(c,1.0f/60.0f)==DQ_OK);
  dq_stats stats{}; assert(dq_context_get_stats(c,&stats)==DQ_OK);
  assert(stats.step_count==1 && stats.particle_count==1 && stats.distance_constraint_count==0 && stats.last_step_nanoseconds>0);
  dq_context_destroy(c);
}

struct AddTests { AddTests() { test_conservation_capacity_and_validation(); test_floor_collision(); test_particle_contact(); test_stats(); } } add_tests;
