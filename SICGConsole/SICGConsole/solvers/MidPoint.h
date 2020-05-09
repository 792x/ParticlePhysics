#pragma once

#include "Solver.h"

class MidPoint : Solver
{
public:
	void simulation_step(std::vector<Particle*> pVector, float dt) override;
};