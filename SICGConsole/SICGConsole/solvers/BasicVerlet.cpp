#include "BasicVerlet.h"

void BasicVerlet::simulation_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {
	
	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	Vec3f newPos, initPos, initVel, initAcc, prevPos;

	// loop through all the particles
	for (int i = 0; i < int(pVector.size()); i++) {
		prevPos = pVector[i]->m_OldPosition;
		initPos = pVector[i]->m_Position;
		initVel = pVector[i]->m_Velocity;
		initAcc = pVector[i]->m_Force / pVector[i]->m_Mass;

		if (start_pos(pVector)) {
			newPos = initPos + initVel * dt + 0.5f * initAcc * dt * dt;
		} else {
			newPos = 2.f * initPos - prevPos + initAcc * dt * dt;
		}

		pVector[i]->m_OldPosition = initPos;
		pVector[i]->m_Position = newPos;
		pVector[i]->m_Velocity = (newPos - initPos) / dt;
	}

}

// TODO: implement in a different, more efficient fashion --> i.e. change a global variable when simulation is being resetted
bool BasicVerlet::start_pos(std::vector<Particle*> pVector) {

	bool start_pos = true, bx, by, bz;

	for (int i = 0; i < int(pVector.size()); i++) {
		bx = (pVector[i]->m_Position[0] == pVector[i]->m_ConstructPos[0]);
		by = (pVector[i]->m_Position[0] == pVector[i]->m_ConstructPos[0]);
		bz = (pVector[i]->m_Position[0] == pVector[i]->m_ConstructPos[0]);

		if (!(bx && by && bz)) {
			start_pos = false;
			break;
		}
	}

	return start_pos;
}