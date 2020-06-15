#pragma once

#include "Solver.h"

class Euler : public Solver
{
public:
	void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) override;
	enum TYPE {
		expl,
		semi,
		impl
	};
	TYPE type;
	Euler(TYPE type);
};