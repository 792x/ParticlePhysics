#pragma once
#include <vector>
#include "../Force.h"
#include "../Particle.h"

class Solver
{
public:
	virtual void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) = 0;
};