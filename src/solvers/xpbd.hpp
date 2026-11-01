#pragma once

#include "core/context.hpp"

void solve_distance_constraints(Particles& particles, std::vector<Constraint>& constraints,
                                uint32_t iterations, float step_size);
