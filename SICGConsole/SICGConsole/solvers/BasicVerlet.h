#pragma once

#include <stdbool.h>

#include "Solver.h"
#include "../Force.h"
#include "../Particle.h"

class BasicVerlet : public Solver
{
public:
    void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) override;
private:
    bool start_pos(std::vector<Particle*> pVector);
};