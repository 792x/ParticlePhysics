#include "Euler.h"

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f initPos = pVector[i]->m_Position;
		Vec3f initVel = pVector[i]->m_Velocity;
		Vec3f initAcc = pVector[i]->m_Force / pVector[i]->m_Mass;

		Vec3f newVelocity = initVel + initAcc * dt;

		Vec3f newPosition = initPos + initVel * dt;

		if (type == semi) {
			Vec3f newPosition = initPos + newVelocity * dt;
		}
		//if (type == impl) {
			//to do
		//}


		pVector[i]->m_Position = newPosition;
		pVector[i]->m_Velocity = newVelocity;
	}
	

}