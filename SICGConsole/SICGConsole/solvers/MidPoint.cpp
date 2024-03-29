#include "Euler.h"
#include "MidPoint.h"

static Euler EulerSolver = Euler(Euler::semi);

void MidPoint::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	std::vector<Vec3f> initPosVec, initVelVec;

	// Store the initial positions and velocities of the particles.
	for (int i = 0; i < int(pVector.size()); i++) {
		initPosVec.push_back(pVector[i]->m_Position);
		initVelVec.push_back(pVector[i]->m_Velocity);
	}

	// Do an Euler step with half the size of the simulations stepsize dt.
	EulerSolver.simulation_step(pVector, fVector, cVector, float(0.5) * dt);

	// Compute the new positions and velocities for all the particles.
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f initPos = initPosVec[i];
		Vec3f initVel = initVelVec[i];

		Vec3f halfWayAcc = pVector[i]->m_Force / pVector[i]->m_Mass;
		Vec3f halfWayVel = initVel + halfWayAcc * 0.5f * dt;

		Vec3f finalVelocity = initVel + halfWayAcc * dt;
		Vec3f finalPosition = initPos + halfWayVel * dt;

		pVector[i]->next_state(finalPosition, finalVelocity);
	}
}