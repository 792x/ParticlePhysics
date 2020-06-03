#pragma once

#include "Solver.h"

class AdaptiveTimeStepper
{
public:
	AdaptiveTimeStepper(float start_step);

	void reset(std::vector<Force*> fVector);
	void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, Solver* solver);

private:
	bool AdaptiveTimeStepper::minor_deformation_check(std::vector<Force*> fVector);
	void spring_deformation_update(std::vector<Force*> fVector);
	void undo_solver_step(std::vector<Particle*> pVector);

	float dt, total_step_dt, min_dt, max_dt;
	float num_succesful_steps;
};