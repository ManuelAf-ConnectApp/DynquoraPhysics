#pragma once

#include "core/context.hpp"

#include <cstdint>

void rebuild_uniform_grid(Particles& particles, float cell_size, std::vector<GridEntry>& entries);
[[nodiscard]] bool same_cell(const GridEntry& entry, int32_t x, int32_t y, int32_t z);
