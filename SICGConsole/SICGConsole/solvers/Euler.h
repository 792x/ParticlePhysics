#pragma once

#include "Solver.h"
#include "../Force.h"
#include "../Particle.h"

class Euler : public Solver
{
public:
	void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) override;
};