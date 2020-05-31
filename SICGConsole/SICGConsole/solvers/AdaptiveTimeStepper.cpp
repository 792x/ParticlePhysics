#pragma once

#include "AdaptiveTimeStepper.h"
#include "../SpringForce.h"

AdaptiveTimeStepper::AdaptiveTimeStepper(float c_dt){
	total_step_dt = c_dt;
	min_dt = c_dt/512; // allows a maximum of 512 intermediate steps
	max_dt = c_dt; // makes AdaptiveTimeStepper useful for resolving instabilities when the constant timestep is too big
	dt = max_dt;
	num_succesful_steps = 0;
}

void AdaptiveTimeStepper::reset(std::vector<Force*> fVector) {
	num_succesful_steps = 0;
	for (Force* f : fVector) {
		SpringForce* sf = dynamic_cast<SpringForce*>(f);
		if (sf != NULL) {
			sf->reset();
		}
	}
}

void AdaptiveTimeStepper::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, Solver* solver) {
	
	float current_step_dt = 0; // variable to keep track of current timestep time 

	while (current_step_dt < total_step_dt) {// Check if the dt of the current is smaller than the target.
		// Check if the timestep can be increased.
		if (num_succesful_steps >= 2) {
			num_succesful_steps = 0;
			dt = 2 * dt;
			if (dt > max_dt) {// decrease dt if necessary
				dt = max_dt;
			}

		}

		// Do the simulation step.
		solver->simulation_step(pVector, fVector, cVector, dt);
		if (minor_deformation_check(fVector) || dt == min_dt) { // check if the deformations are not too big unless the time_step is already minimal
			spring_deformation_update(fVector); // update the spring deformations used by AdaptiveTimeStepper for the next step
			num_succesful_steps++;
			current_step_dt += dt;
			printf("Current dt: %f\n", dt);
		} else { // if the deformations are too big, undo the step by the solver and decrease the timestep
			undo_solver_step(pVector);
			num_succesful_steps = 0;
			dt = 0.5 * dt;
			if (dt < min_dt) {// increase dt if necessary
				dt = min_dt;
			}
		}
	}
}

// Check if the computed distance by the solver is less than 10% from the previous distance.
bool AdaptiveTimeStepper::minor_deformation_check(std::vector<Force*> fVector) {
	for (Force* f : fVector) {
		SpringForce* sf = dynamic_cast<SpringForce*>(f);
		if (sf != NULL) {
			// If the deformation is not minor return false
			if (!sf->deformation_check()) {
				return false;
			}
		}
	}

	return true;
}

void AdaptiveTimeStepper::spring_deformation_update(std::vector<Force*> fVector) {
	for (Force* f : fVector) {
		SpringForce* sf = dynamic_cast<SpringForce*>(f);
		if (sf != NULL) {
			// If we are dealing with a springforce, update the current length
			sf->deformation_update();
		}
	}
}

// Undo the previous step of the solver by resetting the variables of the particles.
void AdaptiveTimeStepper::undo_solver_step(std::vector<Particle*> pVector) {
	for (Particle* p : pVector) {
		p->undo_next_state();
	}
}