#include "spatial/uniform_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
int32_t cell_coordinate(float value, float cell_size) {
  const double minimum = static_cast<double>(std::numeric_limits<int32_t>::min());
  const double maximum = static_cast<double>(std::numeric_limits<int32_t>::max());
  return static_cast<int32_t>(std::clamp(std::floor(static_cast<double>(value) / cell_size), minimum, maximum));
}
}

void rebuild_uniform_grid(Particles& particles, float cell_size, std::vector<GridEntry>& entries) {
  entries.clear();
  if (cell_size <= 0.0f) return;
  for (uint32_t particle = 0; particle < particles.size(); ++particle) {
    entries.push_back({cell_coordinate(particles.px[particle], cell_size),
                       cell_coordinate(particles.py[particle], cell_size),
                       cell_coordinate(particles.pz[particle], cell_size), particle});
  }
  std::sort(entries.begin(), entries.end(), [](const GridEntry& left, const GridEntry& right) {
    if (left.x != right.x) return left.x < right.x;
    if (left.y != right.y) return left.y < right.y;
    if (left.z != right.z) return left.z < right.z;
    return left.particle < right.particle;
  });
}

bool same_cell(const GridEntry& entry, int32_t x, int32_t y, int32_t z) {
  return entry.x == x && entry.y == y && entry.z == z;
}
