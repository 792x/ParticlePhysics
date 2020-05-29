#include "Euler.h"
#include <Eigen/IterativeLinearSolvers>

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulation_step(std::vector<Particle *> pVector,
							std::vector<Force *> fVector,
							std::vector<Constraint *> cVector,
							float dt) {

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);
	if (type==impl) {
		//See 'Implementing the implicit Euler method for mass-spring systems'
		int dimensions = 3;
		// Position vector
		Eigen::VectorXf x0 = Eigen::VectorXf::Zero(pVector.size()*dimensions);
		// Force vector
		Eigen::VectorXf f0 = Eigen::VectorXf::Zero(pVector.size()*dimensions);
		// Velocity vector
		Eigen::VectorXf v0 = Eigen::VectorXf::Zero(pVector.size()*dimensions);
		// Mass matrix
		Eigen::MatrixXf
			M = Eigen::MatrixXf::Zero(pVector.size()*dimensions, pVector.size()*dimensions);
		// Inverse mass matrix
		Eigen::MatrixXf
			W = Eigen::MatrixXf::Zero(pVector.size()*dimensions, pVector.size()*dimensions);

		//TODO: jacobian matrices dfdx and dfdv
		Eigen::MatrixXf
			dfdx = Eigen::MatrixXf::Zero(pVector.size()*dimensions, pVector.size()*dimensions);
		Eigen::MatrixXf
			dfdv = Eigen::MatrixXf::Zero(pVector.size()*dimensions, pVector.size()*dimensions);

		// Fill matrices and vectors for all particles
		for (int i = 0; i < int(pVector.size()); i++) {
			Particle *p = pVector[i/dimensions];
			for (int dim = 0; dim < dimensions; dim++) {
				M(i + dim, i + dim) = p->m_Mass;
				W(i + dim, i + dim) = 1/p->m_Mass;
				x0[i + dim] = p->m_Position[dim];
				f0[i + dim] = p->m_Force[dim];
				v0[i + dim] = p->m_Velocity[dim];
			}
		}

		//(dt*M^-1*df/dv - dt^2*M^-1*df/dx)
		Eigen::MatrixXf lhs = (dt*W*dfdv) - (pow(dt, 2)*W* dfdx);

		// dt*M^-1*(f0 + dt*df/dx*v0)
		Eigen::VectorXf rhs = dt*W*(f0 + dt*dfdx*v0);

		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
		cg.compute(lhs);

		//solve dv
		Eigen::MatrixXf dv = cg.solve(rhs);

		for (int i = 0; i < pVector.size(); i++) {
			Particle *p = pVector[i];
			int index = dimensions*i;
			for (int d = 0; d < dimensions; d++) {
				//TODO: apply result to particles
			}
		}
	} else {
		// loop through all the particles
		for (int i = 0; i < int(pVector.size()); i++) {
			Vec3f initPos = pVector[i]->m_Position;
			Vec3f initVel = pVector[i]->m_Velocity;
			Vec3f initAcc = pVector[i]->m_Force/pVector[i]->m_Mass;

			Vec3f newVelocity = initVel + initAcc*dt;
			Vec3f newPosition = initPos + initVel*dt;

			if (type==semi) {
				newPosition = initPos + newVelocity*dt;
			}

			pVector[i]->next_state(newPosition, newVelocity);
		}
	}
}