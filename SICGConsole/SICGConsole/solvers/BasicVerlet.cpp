#include "BasicVerlet.h"

void BasicVerlet::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {
	
	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	Vec3f newPos, newVel, initPos, initVel, initAcc, prevPos;

	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		prevPos = pVector[i]->m_OldPosition;
		initPos = pVector[i]->m_Position;
		initVel = pVector[i]->m_Velocity;
		initAcc = pVector[i]->m_Force / pVector[i]->m_Mass;

		if (Solver::simulation_reset) {
			newPos = initPos + initVel * dt + 0.5f * initAcc * dt * dt;
			Solver::simulation_reset = false;
		} else {
			newPos = 2.f * initPos - prevPos + initAcc * dt * dt;
		}

		newVel = (newPos - initPos) / dt;

		pVector[i]->next_state(newPos, newVel);
	}

}