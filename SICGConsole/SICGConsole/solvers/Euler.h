#pragma once

#include "Solver.h"
#include "../Particle.h"

class Euler : Solver
{
public:
	void simulation_step(std::vector<Particle*> pVector, float dt) override;
};