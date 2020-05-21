#pragma once

#include "Solver.h"
#include "../Force.h"
#include "../Particle.h"
#include "../Constraint.h"

class RungeKutta : public Solver
{
public:
    void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) override;
};