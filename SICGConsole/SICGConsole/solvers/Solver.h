#pragma once
#include <vector>
#include "../Particle.h"

class Solver
{
public:
	virtual void simulation_step(std::vector<Particle*> pVector, float dt) = 0;
};