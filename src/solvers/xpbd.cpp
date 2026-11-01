#include "solvers/xpbd.hpp"

#include <cmath>
#include <limits>

void solve_distance_constraints(Particles& particles, std::vector<Constraint>& constraints,
                                uint32_t iterations, float step_size) {
  const float step_size_squared = step_size * step_size;
  for (Constraint& constraint : constraints) constraint.lambda = 0.0f;

  for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
    for (Constraint& constraint : constraints) {
      const float dx = particles.px[constraint.a] - particles.px[constraint.b];
      const float dy = particles.py[constraint.a] - particles.py[constraint.b];
      const float dz = particles.pz[constraint.a] - particles.pz[constraint.b];
      const float length_squared = dx * dx + dy * dy + dz * dz;
      if (length_squared <= std::numeric_limits<float>::epsilon()) continue;

      const float length = std::sqrt(length_squared);
      const float inverse_mass_sum = particles.inverse_mass[constraint.a] + particles.inverse_mass[constraint.b];
      const float alpha = constraint.compliance / step_size_squared;
      if (inverse_mass_sum + alpha <= 0.0f) continue;

      const float delta_lambda = (-(length - constraint.rest) - alpha * constraint.lambda) /
                                 (inverse_mass_sum + alpha);
      constraint.lambda += delta_lambda;
      const float correction = delta_lambda / length;
      particles.px[constraint.a] += dx * correction * particles.inverse_mass[constraint.a];
      particles.py[constraint.a] += dy * correction * particles.inverse_mass[constraint.a];
      particles.pz[constraint.a] += dz * correction * particles.inverse_mass[constraint.a];
      particles.px[constraint.b] -= dx * correction * particles.inverse_mass[constraint.b];
      particles.py[constraint.b] -= dy * correction * particles.inverse_mass[constraint.b];
      particles.pz[constraint.b] -= dz * correction * particles.inverse_mass[constraint.b];
    }
  }
}
