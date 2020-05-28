#include "Euler.h"
#include <Eigen/IterativeLinearSolvers>
#include "linearSolver.h"
#include "gfx/mat3.h"

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulation_step(std::vector<Particle *> pVector,
							std::vector<Force *> fVector,
							std::vector<Constraint *> cVector,
							float dt) {

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);
	if (type==impl) {
		int dimensions = 3;
		//See 'Implementing the implicit Euler method for mass-spring systems'
		Eigen::VectorXf x0 = Eigen::VectorXf::Zero(pVector.size() * dimensions);// Position vector
		Eigen::VectorXf f0 = Eigen::VectorXf::Zero(pVector.size() * dimensions);// Force vector
		Eigen::VectorXf v0 = Eigen::VectorXf::Zero(pVector.size() * dimensions);// Velocity vector
		Eigen::MatrixXf	M = Eigen::MatrixXf::Zero(pVector.size() * dimensions, pVector.size() * dimensions);// Mass matrix
		Eigen::MatrixXf	W = Eigen::MatrixXf::Zero(pVector.size() * dimensions, pVector.size() * dimensions);// Inverse mass matrix

		// Fill matrices and vectors for all particles
		for (int i = 0; i < int(pVector.size()); i++) {
			Particle* p = pVector[i / dimensions];
			for (int dim = 0; dim < dimensions; dim++) {
				M(i + dim, i + dim) = p->m_Mass;
				W(i + dim, i + dim) = 1 / p->m_Mass;
				x0[i + dim] = p->m_Position[dim];
				f0[i + dim] = p->m_Force[dim];
				v0[i + dim] = p->m_Velocity[dim];
			}
		}
		
		Eigen::MatrixXf	dfdx = Eigen::MatrixXf::Zero(pVector.size() * dimensions, pVector.size() * dimensions); //Jacobian wrt x
		Eigen::MatrixXf	dfdv = Eigen::MatrixXf::Zero(pVector.size() * dimensions, pVector.size() * dimensions); //Jacobian wrt v

		for (int f = 0; f < fVector.size(); f++) {
			Vec3 dx = fVector[f]->particles[0]->m_Position - fVector[f]->particles[1]->m_Position;
			Mat3 dxtdx = Mat3::outer_product(dx, dx);
			Mat3 ident = Mat3::I();
			double l = sqrt(dx * dx);
			if (l != 0) {
				l = 1.0 / l;
			}
			dxtdx = dxtdx * (l * l);
			fVector[f]->Jx = (dxtdx + (ident - dxtdx) * (1 - fVector[f]->dist * l)) * (fVector[f]->ks);
			fVector[f]->Jv = ident * fVector[f]->kd;
			Particle* p1 = fVector[f]->particles[0];
			Particle* p2 = fVector[f]->particles[1];
			int rIndex = p1->m_Index;
			int cIndex = p2->m_Index;
			for (int r = 0; r < 3; r++) {
				for (int c = 0; c < 3; c++) {
					dfdx(3 * rIndex + r, 3 * cIndex + c) += fVector[f]->Jx(r, c);
					//std::cout << "dfdx (" << std::to_string(3 * rIndex + r)<<"," << std::to_string(3 * cIndex + r) <<") "<< std::to_string(fVector[f]->Jx(r, c))<<"\n";
					dfdv(3 * rIndex + r, 3 * cIndex + c) += fVector[f]->Jv(r, c);
				}
			}

		}
		
		//A*dv=b

		//A = (dt*M^-1*df/dv - dt^2*M^-1*df/dx)
		Eigen::MatrixXf lhs = (dt * W * dfdv) - (pow(dt, 2) * W * dfdx); 

		//b = dt*M^-1*(f0 + dt*df/dx*v0)
		Eigen::VectorXf rhs = dt*W*(f0 + dt*dfdx*v0);

		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
		cg.compute(lhs);

		//solve dv
		Eigen::VectorXf dv = cg.solve(rhs); //dv=NAN? TODO: something is not right with this function/attribute??
		for (int i = 0; i < dv.size(); i++) {
			if (isnan(dv(i))) { dv(i) = 0; }
		}

		for (int i = 0; i < pVector.size(); i++) {
			Particle* p = pVector[i];
			int index = dimensions * i;
			for (int d = 0; d < dimensions; d++) {
				//apply result to particles
				p->m_Velocity[d] += dv(index + d);
				p->m_Position[d] += p->m_Velocity[d] * dt;
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

			pVector[i]->m_Position = newPosition;
			pVector[i]->m_Velocity = newVelocity;
		}
	}
}