#pragma once

#include "../constraints/Constraint.h"
#include "../objects/Particle.h"

class ConstraintSolver {
  public:
	static void solve(std::vector<Particle *> particles, std::vector<Constraint *> constraints, float Ks, float Kd);
};