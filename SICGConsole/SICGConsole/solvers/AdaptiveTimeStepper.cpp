#pragma once

#include "AdaptiveTimeStepper.h"
#include "../SpringForce.h"

AdaptiveTimeStepper::AdaptiveTimeStepper(float start_step){
	start_dt = start_step;
	dt = start_step;
	num_succesful_steps = 0;
}

void AdaptiveTimeStepper::reset(std::vector<Force*> fVector) {
	dt = start_dt;
	num_succesful_steps = 0;
	for (Force* f : fVector) {
		SpringForce* sf = dynamic_cast<SpringForce*>(f);
		if (sf != NULL) {
			sf->reset();
		}
	}
}

void AdaptiveTimeStepper::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, Solver* solver) {
	
	// Check if the timestep can be increased.
	if (num_succesful_steps >= 2) {
		num_succesful_steps = 0;
		dt = 1.05 * dt;
	}

	// Do the simulation step.
	solver->simulation_step(pVector, fVector, cVector, dt);
	if (minor_deformation_check(fVector)) { // check if the deformations are not too big
		spring_deformation_update(fVector);
		num_succesful_steps++;
	}
	else { // if the deformations are too big, undo the step by the solver and decrease the timestep
		undo_solver_step(pVector);
		num_succesful_steps = 0;
		dt = 0.95 * dt;
	}

	printf("Current dt: %f\n", dt);
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


/*
----------------------------------------------------------------------
Routines needed for adaptive step sizing
----------------------------------------------------------------------
*/

//static bool adaptive_time_stepping = true;
//static int num_succesful_steps = 0;
//static float dt;

//// Initialize variables used for the adaptive step sizing.
//static void adaptive_step_init() {
//	dt = c_dt; // Start with the predifined constant time step
//	num_succesful_steps = 0; // Set the number of already done succesful steps with the current dt
//
//	// Initialize the springs
//	for (Force* f : fVector) {
//		SpringForce* sf = dynamic_cast<SpringForce*>(f);
//		if (sf != NULL) {
//			sf->deformation_init();
//		}
//	}
//}

//// Set the new spring distance for each spring.
//static void spring_deformation_update() {
//	for (Force* f : fVector) {
//		SpringForce* sf = dynamic_cast<SpringForce*>(f);
//		if (sf != NULL) {
//			sf->deformation_update();
//		}
//	}
//}
//
//// Undo the previous step of the solver by resetting the variables of the particles.
//static void undo_step() {
//	for (Particle* p : pVector) {
//		p->undo_next_state();
//	}
//}