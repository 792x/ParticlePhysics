#include "../Particle.h"
#include "../include/gfx/vec2.h"

#include "Euler.h"

void Euler::simulation_step(std::vector<Particle*> pVector, float dt) {
	
	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		Vec3f initPos = pVector[i]->m_Position;
		Vec3f initVel = pVector[i]->m_Velocity;
		Vec3f initAcc = pVector[i]->m_Force / pVector[i]->m_Mass;

		Vec3f finalVelocity = initVel + initAcc * dt;
		Vec3f finalPosition = initPos + initVel * dt + float(0.5) * initAcc * dt * dt;
		
		pVector[i]->m_Position = finalPosition;
		pVector[i]->m_Velocity = finalVelocity;
	}

}