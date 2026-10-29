#include <dynquora/physics.h>
#include <raylib.h>

#include <cmath>
#include <cstdio>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr float kScale = 90.0f;
constexpr float kDt = 1.0f / 60.0f;

Vector2 screen(dq_vec3 p) { return {kWidth * 0.5f + p.x * kScale, kHeight * 0.72f - p.y * kScale}; }

dq_context* create_scene(dq_particle_id* ids, int count) {
  dq_context_desc desc{};
  desc.struct_size = sizeof desc;
  desc.max_particles = static_cast<uint32_t>(count);
  desc.max_distance_constraints = static_cast<uint32_t>(count - 1);
  desc.solver_iterations = 12;
  desc.substeps = 2;
  desc.gravity = {0.0f, -9.81f, 0.0f};
  desc.linear_damping = 0.002f;
  desc.grid_cell_size = 0.5f;
  desc.enable_floor = 1;
  desc.floor_height = -2.5f;
  desc.floor_restitution = 0.1f;
  desc.floor_friction = 0.15f;
  desc.particle_radius = 0.06f;
  dq_context* context = nullptr;
  if (dq_context_create(&desc, &context) != DQ_OK) return nullptr;

  for (int i = 0; i < count; ++i) {
    dq_particle_desc particle{};
    particle.position = {-3.0f + i * 0.32f, 2.0f, 0.0f};
    particle.inverse_mass = (i == 0) ? 0.0f : 1.0f;
    if (dq_context_add_particle(context, &particle, &ids[i]) != DQ_OK) { dq_context_destroy(context); return nullptr; }
  }
  for (int i = 1; i < count; ++i) {
    dq_distance_constraint_desc constraint{ids[i - 1], ids[i], 0.32f, 0.0f};
    dq_constraint_id ignored{};
    if (dq_context_add_distance_constraint(context, &constraint, &ignored) != DQ_OK) { dq_context_destroy(context); return nullptr; }
  }
  return context;
}

void draw_scene(dq_context* context, const dq_particle_id* ids, int count) {
  dq_particle_view view{};
  if (dq_particles_acquire(context, &view) != DQ_OK) return;
  for (int i = 1; i < count; ++i) DrawLineV(screen(view.positions[ids[i - 1]]), screen(view.positions[ids[i]]), SKYBLUE);
  for (uint32_t i = 0; i < view.count; ++i) {
    const bool pinned = i == 0;
    DrawCircleV(screen(view.positions[i]), pinned ? 8.0f : 6.0f, pinned ? ORANGE : RAYWHITE);
  }
  dq_particles_release(context, view.generation);
}
}

int main() {
  constexpr int count = 24;
  dq_particle_id ids[count]{};
  dq_context* context = create_scene(ids, count);
  if (!context) return 1;

  InitWindow(kWidth, kHeight, "Dynquora Physics — XPBD diagnostic viewer");
  SetTargetFPS(60);
  bool paused = false;
  while (!WindowShouldClose()) {
    if (IsKeyPressed(KEY_SPACE)) paused = !paused;
    if (IsKeyPressed(KEY_R)) { dq_context_destroy(context); context = create_scene(ids, count); if (!context) break; }
    const bool one_step = IsKeyPressed(KEY_N);
    if (!paused || one_step) dq_step(context, kDt);

    BeginDrawing();
    ClearBackground({18, 23, 32, 255});
    for (int y = 0; y < kHeight; y += 45) DrawLine(0, y, kWidth, y, {30, 38, 52, 255});
    DrawLine(0, static_cast<int>(kHeight * 0.72f), kWidth, static_cast<int>(kHeight * 0.72f), {65, 75, 90, 255});
    draw_scene(context, ids, count);
    DrawText("Dynquora XPBD | rope scene", 20, 18, 24, RAYWHITE);
    DrawText("Space: pause/resume   N: step   R: reset", 20, 48, 18, LIGHTGRAY);
    dq_stats stats{}; dq_context_get_stats(context, &stats);
    DrawText(TextFormat("FPS: %d | particles: %d | constraints: %d | contacts: %d | %.3f ms/step | %s", GetFPS(), count, count - 1, stats.contact_pair_count, static_cast<double>(stats.last_step_nanoseconds) / 1'000'000.0, paused ? "paused" : "running"), 20, 76, 18, paused ? ORANGE : LIME);
    EndDrawing();
  }
  dq_context_destroy(context);
  CloseWindow();
}
