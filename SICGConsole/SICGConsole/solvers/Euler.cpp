#include "../Particle.h"
#include "../include/gfx/vec2.h"

#include "Euler.h"

void Euler::simulation_step(std::vector<Particle*> pVector, float dt) {
	
	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f oldPosition = pVector[i]->m_Position;
		Vec3f oldVelocity = pVector[i]->m_Velocity;
		Vec3f acceleration = pVector[i]->m_Force/pVector[i]->m_Mass;

		Vec3f newVelocity = oldVelocity + acceleration * dt;
		Vec3f newPosition = oldPosition + newVelocity * dt;
		
		pVector[i]->m_Velocity = newVelocity;
		pVector[i]->m_Position = newPosition;
	}

}