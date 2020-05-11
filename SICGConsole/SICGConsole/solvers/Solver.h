#pragma once
#include <vector>
#include "../Force.h"
#include "../Particle.h"

class Solver
{
public:
	virtual void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) = 0;

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
};