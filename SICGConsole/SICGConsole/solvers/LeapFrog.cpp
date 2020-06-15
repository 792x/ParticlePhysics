#pragma once

#include "LeapFrog.h"
#include "RungeKutta.h"

static RungeKutta RungeKuttaSolver;

void LeapFrog::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {

	// If the simulation was resetted, initialize a halfway velocity first
	if (Solver::simulation_reset) {

		std::vector<Vec3f> initPosVec;

		// Store the initial positions
		for (int i = 0; i < int(pVector.size()); i++) {
			initPosVec.push_back(pVector[i]->m_Position);
		}

		// Use the Runge Kutta scheme to get the velocities halfway the first step
		RungeKuttaSolver.simulation_step(pVector, fVector, cVector, 0.5f * dt);

		// Store the values, i for the positions, i+0.5 for the velocities
		for (int i = 0; i < int(pVector.size()); i++) {
			pVector[i]->next_state(initPosVec[i], pVector[i]->m_Velocity);
		}

		Solver::simulation_reset = false;
	}

	/* Do the leap frog integration */
	// Update the positions
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f newPosition = pVector[i]->m_Position + pVector[i]->m_Velocity * dt;
		pVector[i]->m_Position = newPosition;
	}

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	// Update the velocities
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f acceleration = pVector[i]->m_Force / pVector[i]->m_Mass;
		Vec3f newVelocity = pVector[i]->m_Velocity + acceleration * dt;

		// Set the next state
		pVector[i]->next_state(pVector[i]->m_Position, newVelocity);
	}
}