#include <dynquora/physics.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <vector>

struct Constraint { uint32_t a, b; float rest, compliance, lambda; };
struct GridEntry { int64_t key; int32_t x, y; uint32_t particle; };
struct ParticleContact { uint32_t a, b; float lambda, nx, ny, nz, incoming_normal_speed; uint64_t last_seen_step; };
struct Particles {
  std::vector<float> px, py, pz, prev_x, prev_y, prev_z, vx, vy, vz, inverse_mass;
  std::vector<dq_vec3> published_positions, published_velocities;
  void reserve(uint32_t capacity) {
    px.reserve(capacity); py.reserve(capacity); pz.reserve(capacity); prev_x.reserve(capacity); prev_y.reserve(capacity); prev_z.reserve(capacity);
    vx.reserve(capacity); vy.reserve(capacity); vz.reserve(capacity); inverse_mass.reserve(capacity);
    published_positions.reserve(capacity); published_velocities.reserve(capacity);
  }
  size_t size() const { return px.size(); }
  void add(dq_particle_desc d) {
    px.push_back(d.position.x); py.push_back(d.position.y); pz.push_back(d.position.z);
    prev_x.push_back(d.position.x); prev_y.push_back(d.position.y); prev_z.push_back(d.position.z);
    vx.push_back(d.velocity.x); vy.push_back(d.velocity.y); vz.push_back(d.velocity.z); inverse_mass.push_back(d.inverse_mass);
  }
};
struct dq_context {
  uint32_t max_particles, max_constraints, iterations, substeps;
  dq_vec3 gravity;
  float linear_damping;
  float grid_cell_size, floor_height, floor_restitution, floor_friction, particle_radius, particle_contact_compliance, particle_restitution, particle_friction;
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

namespace {

bool finite_vec(dq_vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
int32_t cell_coordinate(float value, float cell_size) {
  const double minimum = static_cast<double>(std::numeric_limits<int32_t>::min());
  const double maximum = static_cast<double>(std::numeric_limits<int32_t>::max());
  return static_cast<int32_t>(std::clamp(std::floor(static_cast<double>(value) / cell_size), minimum, maximum));
}
int64_t cell_key(int32_t cx, int32_t cy) {
  return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy);
}
int32_t neighbouring_cell_coordinate(int32_t value, int offset) {
  if (offset < 0 && value == std::numeric_limits<int32_t>::min()) return value;
  if (offset > 0 && value == std::numeric_limits<int32_t>::max()) return value;
  return static_cast<int32_t>(value + offset);
}
void set_error(dq_context* c, const char* msg) { if (c) c->error = msg; }
bool valid_desc(const dq_context_desc& d) {
  return d.max_particles > 0 && d.solver_iterations > 0 && d.substeps > 0 &&
    d.backend == DQ_BACKEND_CPU && finite_vec(d.gravity) && std::isfinite(d.linear_damping) &&
    d.linear_damping >= 0.0f && d.linear_damping <= 1.0f &&
    std::isfinite(d.grid_cell_size) && d.grid_cell_size >= 0.0f && d.enable_floor <= 1 &&
    std::isfinite(d.particle_radius) && d.particle_radius >= 0.0f && std::isfinite(d.particle_contact_compliance) && d.particle_contact_compliance >= 0.0f && std::isfinite(d.particle_restitution) && d.particle_restitution >= 0.0f && d.particle_restitution <= 1.0f && std::isfinite(d.particle_friction) && d.particle_friction >= 0.0f && d.particle_friction <= 1.0f &&
    (!d.enable_floor || (std::isfinite(d.floor_height) && std::isfinite(d.floor_restitution) && std::isfinite(d.floor_friction) && d.floor_restitution >= 0.0f && d.floor_restitution <= 1.0f && d.floor_friction >= 0.0f && d.floor_friction <= 1.0f)) &&
    ((d.allocator.allocate == nullptr && d.allocator.deallocate == nullptr) ||
     (d.allocator.allocate != nullptr && d.allocator.deallocate != nullptr));
}
}

extern "C" {
uint32_t DQ_CALL dq_get_api_version(void) { return (DQ_API_VERSION_MAJOR << 16u) | (DQ_API_VERSION_MINOR << 8u) | DQ_API_VERSION_PATCH; }
const char* DQ_CALL dq_result_string(dq_result r) {
  static const char* const text[] = {"OK", "invalid argument", "out of memory", "capacity exceeded", "invalid state", "unsupported backend", "busy", "numerical error", "internal error"};
  return (r >= DQ_OK && r <= DQ_INTERNAL_ERROR) ? text[r] : "unknown result";
}
dq_result DQ_CALL dq_context_create(const dq_context_desc* d, dq_context** out) {
  if (!d || !out || d->struct_size < sizeof(dq_context_desc) || !valid_desc(*d)) return DQ_INVALID_ARGUMENT;
  /* Custom allocators are part of the ABI, but await allocator-aware SoA storage. */
  if (d->allocator.allocate != nullptr) return DQ_UNSUPPORTED_BACKEND;
  *out = nullptr;
  try {
    auto c = std::make_unique<dq_context>();
    c->max_particles=d->max_particles; c->max_constraints=d->max_distance_constraints; c->iterations=d->solver_iterations; c->substeps=d->substeps; c->gravity=d->gravity; c->linear_damping=d->linear_damping; c->grid_cell_size=d->grid_cell_size; c->floor_enabled=d->enable_floor != 0; c->floor_height=d->floor_height; c->floor_restitution=d->floor_restitution; c->floor_friction=d->floor_friction; c->particle_radius=d->particle_radius; c->particle_contact_compliance=d->particle_contact_compliance; c->particle_restitution=d->particle_restitution; c->particle_friction=d->particle_friction;
    c->particles.reserve(d->max_particles); c->constraints.reserve(d->max_distance_constraints); c->grid_entries.reserve(d->max_particles); c->particle_contacts.reserve(d->max_particle_contacts ? d->max_particle_contacts : d->max_particles);
    *out = c.release(); return DQ_OK;
  } catch (const std::bad_alloc&) { return DQ_OUT_OF_MEMORY; } catch (...) { return DQ_INTERNAL_ERROR; }
}
void DQ_CALL dq_context_destroy(dq_context* c) { delete c; }
dq_result DQ_CALL dq_context_add_particle(dq_context* c, const dq_particle_desc* d, dq_particle_id* id) {
  if (!c || !d || !id || !finite_vec(d->position) || !finite_vec(d->velocity) || !std::isfinite(d->inverse_mass) || d->inverse_mass < 0) return DQ_INVALID_ARGUMENT;
  if (c->view_held) return DQ_BUSY;
  if (c->particles.size() == c->max_particles) { set_error(c,"particle capacity exhausted"); return DQ_CAPACITY_EXCEEDED; }
  *id=static_cast<uint32_t>(c->particles.size()); c->particles.add(*d); return DQ_OK;
}
dq_result DQ_CALL dq_context_add_distance_constraint(dq_context* c, const dq_distance_constraint_desc* d, dq_constraint_id* id) {
  if (!c || !d || !id || d->first >= c->particles.size() || d->second >= c->particles.size() || d->first == d->second || !std::isfinite(d->rest_length) || !std::isfinite(d->compliance) || d->rest_length < 0 || d->compliance < 0) return DQ_INVALID_ARGUMENT;
  if (c->view_held) return DQ_BUSY;
  if (c->constraints.size() == c->max_constraints) return DQ_CAPACITY_EXCEEDED;
  *id=static_cast<uint32_t>(c->constraints.size()); c->constraints.push_back({d->first,d->second,d->rest_length,d->compliance,0}); return DQ_OK;
}
dq_result DQ_CALL dq_step(dq_context* c, float dt) {
  if (!c || !std::isfinite(dt) || dt <= 0) return DQ_INVALID_ARGUMENT;
  if (c->view_held) return DQ_BUSY;
  const auto step_start = std::chrono::steady_clock::now();
  c->particle_contacts.erase(std::remove_if(c->particle_contacts.begin(), c->particle_contacts.end(), [c](const ParticleContact& contact) {
    return contact.last_seen_step + 4 < c->stats.step_count;
  }), c->particle_contacts.end());
  c->stats.contact_pair_count = 0;
  c->stats.dropped_contact_count = 0;
  c->stats.maximum_penetration = 0.0f;
  const float h=dt/static_cast<float>(c->substeps); const float h2=h*h;
  for (uint32_t s=0;s<c->substeps;++s) {
    Particles& p=c->particles;
    for (size_t i=0;i<p.size();++i) { p.prev_x[i]=p.px[i]; p.prev_y[i]=p.py[i]; p.prev_z[i]=p.pz[i]; if(p.inverse_mass[i]>0) { p.vx[i]=(p.vx[i]+c->gravity.x*h)*(1.0f-c->linear_damping); p.vy[i]=(p.vy[i]+c->gravity.y*h)*(1.0f-c->linear_damping); p.vz[i]=(p.vz[i]+c->gravity.z*h)*(1.0f-c->linear_damping); p.px[i]+=p.vx[i]*h; p.py[i]+=p.vy[i]*h; p.pz[i]+=p.vz[i]*h; } }
    c->grid_entries.clear();
    if (c->grid_cell_size > 0.0f) {
      for (uint32_t i = 0; i < p.size(); ++i) { const int32_t x=cell_coordinate(p.px[i],c->grid_cell_size), y=cell_coordinate(p.py[i],c->grid_cell_size); c->grid_entries.push_back({cell_key(x,y),x,y,i}); }
      std::sort(c->grid_entries.begin(), c->grid_entries.end(), [](const GridEntry& a, const GridEntry& b) { return a.key < b.key || (a.key == b.key && a.particle < b.particle); });
    }
    for (auto& q:c->constraints) q.lambda=0;
    for(uint32_t it=0;it<c->iterations;++it) for(auto& q:c->constraints) {
      const float dx=p.px[q.a]-p.px[q.b], dy=p.py[q.a]-p.py[q.b], dz=p.pz[q.a]-p.pz[q.b]; float length2=dx*dx+dy*dy+dz*dz; if(length2 <= std::numeric_limits<float>::epsilon()) continue;
      float length=std::sqrt(length2), w=p.inverse_mass[q.a]+p.inverse_mass[q.b], alpha=q.compliance/h2;
      if(w+alpha <= 0) continue; float change=(-(length-q.rest)-alpha*q.lambda)/(w+alpha); q.lambda+=change; const float correction=change/length;
      p.px[q.a]+=dx*correction*p.inverse_mass[q.a]; p.py[q.a]+=dy*correction*p.inverse_mass[q.a]; p.pz[q.a]+=dz*correction*p.inverse_mass[q.a]; p.px[q.b]-=dx*correction*p.inverse_mass[q.b]; p.py[q.b]-=dy*correction*p.inverse_mass[q.b]; p.pz[q.b]-=dz*correction*p.inverse_mass[q.b];
    }
    if (c->particle_radius > 0.0f && c->grid_cell_size > 0.0f) {
      const float minimum_distance = 2.0f * c->particle_radius;
      for (const GridEntry& entry : c->grid_entries) for (int oy=-1; oy<=1; ++oy) for (int ox=-1; ox<=1; ++ox) {
        const int64_t neighbour_key=cell_key(neighbouring_cell_coordinate(entry.x,ox),neighbouring_cell_coordinate(entry.y,oy));
        auto first=std::lower_bound(c->grid_entries.begin(),c->grid_entries.end(),neighbour_key,[](const GridEntry& e,int64_t key){ return e.key<key; });
        for (; first != c->grid_entries.end() && first->key == neighbour_key; ++first) {
          const uint32_t a=entry.particle, b=first->particle; if (b <= a) continue;
          float dx=p.px[a]-p.px[b], dy=p.py[a]-p.py[b], dz=p.pz[a]-p.pz[b]; float d2=dx*dx+dy*dy+dz*dz;
          if (d2 >= minimum_distance*minimum_distance) continue;
          ++c->stats.contact_pair_count;
          float d=std::sqrt(d2); if (d <= std::numeric_limits<float>::epsilon()) { d=1.0e-6f; dx=(a&1u)?d:-d; dy=dz=0.0f; }
          c->stats.maximum_penetration = std::max(c->stats.maximum_penetration, minimum_distance - d);
          const float inverse_sum=p.inverse_mass[a]+p.inverse_mass[b]; if (inverse_sum <= 0.0f) continue;
          ParticleContact* contact = nullptr;
          for (ParticleContact& cached : c->particle_contacts) if (cached.a == a && cached.b == b) { contact = &cached; break; }
          if (!contact) {
            if (c->particle_contacts.size() == c->particle_contacts.capacity()) { ++c->stats.dropped_contact_count; continue; }
            c->particle_contacts.push_back({a, b, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0}); contact = &c->particle_contacts.back();
          }
          contact->last_seen_step = c->stats.step_count + 1;
          contact->nx=dx/d; contact->ny=dy/d; contact->nz=dz/d;
          const float rvx=p.vx[a]-p.vx[b], rvy=p.vy[a]-p.vy[b], rvz=p.vz[a]-p.vz[b];
          contact->incoming_normal_speed=rvx*contact->nx+rvy*contact->ny+rvz*contact->nz;
          const float alpha=c->particle_contact_compliance/h2;
          const float constraint=d-minimum_distance;
          const float old_lambda=contact->lambda;
          contact->lambda=std::max(0.0f, old_lambda+(-constraint-alpha*old_lambda)/(inverse_sum+alpha));
          const float correction=(contact->lambda-old_lambda)/d;
          p.px[a]+=dx*correction*p.inverse_mass[a]; p.py[a]+=dy*correction*p.inverse_mass[a]; p.pz[a]+=dz*correction*p.inverse_mass[a]; p.px[b]-=dx*correction*p.inverse_mass[b]; p.py[b]-=dy*correction*p.inverse_mass[b]; p.pz[b]-=dz*correction*p.inverse_mass[b];
        }
      }
    }
    for(size_t i=0;i<p.size();++i) if(p.inverse_mass[i]>0) { p.vx[i]=(p.px[i]-p.prev_x[i])/h; p.vy[i]=(p.py[i]-p.prev_y[i])/h; p.vz[i]=(p.pz[i]-p.prev_z[i])/h; }
    for (const ParticleContact& contact : c->particle_contacts) if (contact.last_seen_step == c->stats.step_count + 1) {
      const float inverse_sum=p.inverse_mass[contact.a]+p.inverse_mass[contact.b]; if (inverse_sum <= 0.0f) continue;
      float rvx=p.vx[contact.a]-p.vx[contact.b], rvy=p.vy[contact.a]-p.vy[contact.b], rvz=p.vz[contact.a]-p.vz[contact.b];
      const float normal_speed=rvx*contact.nx+rvy*contact.ny+rvz*contact.nz;
      const float target_speed=contact.incoming_normal_speed < 0.0f ? -contact.incoming_normal_speed*c->particle_restitution : 0.0f;
      if (normal_speed < target_speed) { const float impulse=(target_speed-normal_speed)/inverse_sum; p.vx[contact.a]+=contact.nx*impulse*p.inverse_mass[contact.a]; p.vy[contact.a]+=contact.ny*impulse*p.inverse_mass[contact.a]; p.vz[contact.a]+=contact.nz*impulse*p.inverse_mass[contact.a]; p.vx[contact.b]-=contact.nx*impulse*p.inverse_mass[contact.b]; p.vy[contact.b]-=contact.ny*impulse*p.inverse_mass[contact.b]; p.vz[contact.b]-=contact.nz*impulse*p.inverse_mass[contact.b]; }
      const float tangent_x=rvx-normal_speed*contact.nx, tangent_y=rvy-normal_speed*contact.ny, tangent_z=rvz-normal_speed*contact.nz;
      p.vx[contact.a]-=tangent_x*c->particle_friction*p.inverse_mass[contact.a]/inverse_sum; p.vy[contact.a]-=tangent_y*c->particle_friction*p.inverse_mass[contact.a]/inverse_sum; p.vz[contact.a]-=tangent_z*c->particle_friction*p.inverse_mass[contact.a]/inverse_sum;
      p.vx[contact.b]+=tangent_x*c->particle_friction*p.inverse_mass[contact.b]/inverse_sum; p.vy[contact.b]+=tangent_y*c->particle_friction*p.inverse_mass[contact.b]/inverse_sum; p.vz[contact.b]+=tangent_z*c->particle_friction*p.inverse_mass[contact.b]/inverse_sum;
    }
    if (c->floor_enabled) for (size_t i = 0; i < p.size(); ++i) if (p.inverse_mass[i] > 0.0f && p.py[i] < c->floor_height) {
      p.py[i] = c->floor_height;
      if (p.vy[i] < 0.0f) p.vy[i] = -p.vy[i] * c->floor_restitution;
      p.vx[i] *= 1.0f - c->floor_friction;
      p.vz[i] *= 1.0f - c->floor_friction;
    }
  }
  for (size_t i=0;i<c->particles.size();++i) if (!std::isfinite(c->particles.px[i]) || !std::isfinite(c->particles.py[i]) || !std::isfinite(c->particles.pz[i])) { set_error(c, "non-finite position produced by solver"); return DQ_NUMERICAL_ERROR; }
  ++c->generation;
  ++c->stats.step_count;
  c->stats.last_step_nanoseconds = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - step_start).count());
  c->stats.particle_count = static_cast<uint32_t>(c->particles.size());
  c->stats.distance_constraint_count = static_cast<uint32_t>(c->constraints.size());
  c->stats.grid_entry_count = static_cast<uint32_t>(c->grid_entries.size());
  c->stats.cached_contact_count = static_cast<uint32_t>(c->particle_contacts.size());
  return DQ_OK;
}
dq_result DQ_CALL dq_particles_acquire(dq_context* c, dq_particle_view* v) { if(!c||!v) return DQ_INVALID_ARGUMENT; if(c->view_held) return DQ_BUSY; Particles& p=c->particles; p.published_positions.resize(p.size()); p.published_velocities.resize(p.size()); for(size_t i=0;i<p.size();++i) { p.published_positions[i]={p.px[i],p.py[i],p.pz[i]}; p.published_velocities[i]={p.vx[i],p.vy[i],p.vz[i]}; } c->view_held=true; *v={p.published_positions.data(),p.published_velocities.data(),static_cast<uint32_t>(p.size()),c->generation}; return DQ_OK; }
dq_result DQ_CALL dq_particles_release(dq_context* c, uint64_t g) { if(!c || !c->view_held || g != c->generation) return DQ_INVALID_STATE; c->view_held=false; return DQ_OK; }
dq_result DQ_CALL dq_context_last_error(const dq_context* c, char* b, size_t n, size_t* required) { if(!c) return DQ_INVALID_ARGUMENT; const size_t need=c->error.size()+1; if(required) *required=need; if(!b) return n==0?DQ_OK:DQ_INVALID_ARGUMENT; if(n<need) return DQ_CAPACITY_EXCEEDED; std::memcpy(b,c->error.c_str(),need); return DQ_OK; }
dq_result DQ_CALL dq_context_get_stats(const dq_context* c, dq_stats* out) { if (!c || !out) return DQ_INVALID_ARGUMENT; *out = c->stats; return DQ_OK; }
}
