#pragma once

#include "core/context.hpp"

void solve_particle_contacts(dq_context& context, float step_size);
void resolve_contact_velocities(dq_context& context);
