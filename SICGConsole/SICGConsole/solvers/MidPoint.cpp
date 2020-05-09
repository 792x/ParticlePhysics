#include "MidPoint.h"

void MidPoint::simulation_step(std::vector<Particle*> pVector, float dt) {

	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f oldPosition = pVector[i]->m_Position;
		Vec3f oldVelocity = pVector[i]->m_Velocity;
		Vec3f acc = 0; // Force / m; TODO: implement force

		/*pVector[i]->m_Velocity = acc * dt;
		pVector[i]->m_Position = oldPosition + dt * ;*/
	}

}