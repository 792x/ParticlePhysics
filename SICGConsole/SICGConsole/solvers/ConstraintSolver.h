#pragma once

#include "../Constraint.h"
#include "../Particle.h"

class ConstraintSolver {
  public:
	static void solve(std::vector<Particle *> particles, std::vector<Constraint *> constraints, float Ks, float Kd);
};