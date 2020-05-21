#include "Euler.h"
#include "RungeKutta.h"
#include "ConstraintSolver.h"

void RungeKutta::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {

	// Declaration position, velocity and acceleration vectors.
	std::vector<Vec3f> k1PosVec, k1VelVec, k1AccVec;
	std::vector<Vec3f> k2PosVec, k2VelVec, k2AccVec;
	std::vector<Vec3f> k3PosVec, k3VelVec, k3AccVec;
	std::vector<Vec3f> k4PosVec, k4VelVec, k4AccVec;

	/* K1 */
	// Loop through all the particles and set k1 values.
	for (int i = 0; i < int(pVector.size()); i++) {
		k1PosVec.push_back(pVector[i]->m_Position);
		k1VelVec.push_back(pVector[i]->m_Velocity);
		k1AccVec.push_back(pVector[i]->m_Force / pVector[i]->m_Mass);
	}

	/* K2 */
	// Loop through all particles and set the new positions and velocities (k2).
	for (int i = 0; i < int(pVector.size()); i++) {
		k2VelVec.push_back(k1VelVec[i] + k1AccVec[i] * 0.5f * dt);
		k2PosVec.push_back(k1PosVec[i] + k1VelVec[i] * 0.5f * dt);

		pVector[i]->m_Position = k2PosVec[i];
	}

	compute_forces(fVector);
	ConstraintSolver::solve(pVector, cVector, 100.0f, 10.0f);

	// Compute the accelerations at the new points (k2).
	for (int i = 0; i < int(pVector.size()); i++) {
		k2AccVec.push_back(pVector[i]->m_Force / pVector[i]->m_Mass);
	}

	/* K3 */
	// Loop through all particles and set the new positions and velocities (k3).
	for (int i = 0; i < int(pVector.size()); i++) {
		k3VelVec.push_back(k1VelVec[i] + k2AccVec[i] * 0.5f * dt);
		k3PosVec.push_back(k1PosVec[i] + k2VelVec[i] * 0.5f * dt);

		pVector[i]->m_Position = k3PosVec[i];
	}

	compute_forces(fVector);

	// Compute the accelerations at the new points (k3).
	for (int i = 0; i < int(pVector.size()); i++) {
		k3AccVec.push_back(pVector[i]->m_Force / pVector[i]->m_Mass);
	}

	/* K4 */
	// Loop through all particles and set the new positions and velocities (k4).
	for (int i = 0; i < int(pVector.size()); i++) {
		k4VelVec.push_back(k1VelVec[i] + k3AccVec[i] * dt);
		k4PosVec.push_back(k1PosVec[i] + k3VelVec[i] * dt);

		pVector[i]->m_Position = k4PosVec[i];
	}

	compute_forces(fVector);

	// Compute the accelerations at the new points (k4).
	for (int i = 0; i < int(pVector.size()); i++) {
		k4AccVec.push_back(pVector[i]->m_Force / pVector[i]->m_Mass);
	}

	/* Compute and write the final values. */
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f newVelocity = k1VelVec[i] + (1.f / 6.f) * dt * (k1AccVec[i] + 2.f * k2AccVec[i] + 2.f * k3AccVec[i] + k4AccVec[i]);
		Vec3f newPosition = k1PosVec[i] + (1.f / 6.f) * dt * (k1VelVec[i] + 2.f * k2VelVec[i] + 2.f * k3VelVec[i] + k4VelVec[i]);

		pVector[i]->m_Velocity = newVelocity;
		pVector[i]->m_Position = newPosition;
	}

}