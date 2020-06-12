#pragma once
#include <vector>
#include "../forces/Force.h"
#include "../objects/Particle.h"
#include "ConstraintSolver.h"

class Solver
{
public:
	virtual void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) = 0;

	void compute_forces(std::vector<Force*> fVector) {
		// Reset all the forces.
		for (Force* f : fVector) {
			f->reset();
		}

		// Compute and apply all the new forces.
		for (Force* f : fVector) {
			f->apply();
		}
	}

	 void compute_constraints(std::vector<Particle*> pVector, std::vector<Constraint*> cVector) {
		// Compute the constraint forces.
		ConstraintSolver::solve(pVector, cVector, 100.0f, 10.0f);
	 };

	 static bool simulation_reset;
};
