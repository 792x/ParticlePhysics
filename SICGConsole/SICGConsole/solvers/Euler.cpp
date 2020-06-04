#include "Euler.h"
#include "linearSolver.h"
#include <Eigen/IterativeLinearSolvers>
#include "ConstraintSolver.h"
#include "../SpringForce.h"

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulation_step(std::vector<Particle *> pVector,
							std::vector<Force *> fVector,
							std::vector<Constraint *> cVector,
							float dt) {

	// Compute forces + constraints
	compute_forces(fVector);
	compute_constraints(pVector, cVector);

	if (type==impl) {
		int dimensions = 3*pVector.size();
		// Mass matrix
		std::vector<std::vector<float>>
			M = std::vector<std::vector<float>>(dimensions,
												std::vector<float>(dimensions, 0));
		std::vector<double> dv(dimensions, 0);
		std::vector<double> dx(dimensions, 0);
		std::vector<double> rhs(dimensions, 0);

		// Fill mass matrix
		for (int i = 0; i < dimensions; i++) {
			for (int j = 0; j < dimensions; j++) {
				if (i==j) {
					M[i][j] = pVector[i/3]->m_Mass;
				}
			}
		}

		for (Force* f : fVector) {
			// Only for SpringForce
			SpringForce* i = dynamic_cast<SpringForce*>(f);
			if (i != NULL) {
				std::vector<std::vector<float>> j = i->jacobian();
				std::vector<std::vector<float>>
					jd = ConstraintSolver::multiply_matrix_by_scalar(j, pow(dt, 2));
				std::vector<std::vector<float>>
					jd_min = ConstraintSolver::multiply_matrix_by_scalar(jd, -1);
				int p1 = i->particles[0]->m_Index;
				int p2 = i->particles[1]->m_Index;

				M[p1][p2] += jd_min[0][0];
				M[p1][p2 + 1] += jd_min[0][1];
				M[p1 + 1][p2] += jd_min[1][0];
				M[p1 + 1][p2 + 1] += jd_min[1][1];

				M[p2][p1] += jd[0][0];
				M[p2][p1 + 1] += jd[0][1];
				M[p2 + 1][p1] += jd[1][0];
				M[p2 + 1][p1 + 1] += jd[1][1];

				std::vector<float> v1 = std::vector<float>(3, 0);
				std::vector<float> v2 = std::vector<float>(3, 0);

				v1[0] = i->particles[0]->m_Velocity[0];
				v1[1] = i->particles[0]->m_Velocity[1];
				v2[0] = i->particles[1]->m_Velocity[0];
				v2[1] = i->particles[1]->m_Velocity[1];

				std::vector<float> a1 = ConstraintSolver::multiply_matrix_by_vector(j, v1);
				std::vector<float> a2 = ConstraintSolver::multiply_matrix_by_vector(j, v2);

				std::vector<float> r1 = ConstraintSolver::multiply_vector_by_scalar(a1, pow(dt, 2));
				std::vector<float> r2 = ConstraintSolver::multiply_vector_by_scalar(a2, pow(dt, 2));

				rhs[p1] = r1[0];
				rhs[p1 + 1] = r1[1];
				rhs[p2] = r2[0];
				rhs[p2 + 1] = r2[1];
			}
		}

		for (int i = 0; i < dimensions; i += 3) {
			rhs[i] += dt*pVector[i/3]->m_Force[0];
			rhs[i + 1] += dt*pVector[i/3]->m_Force[1];
		}

		// Only used for provided linear solver
		// implicitMatrix M_I(&M);
		// int max_steps = 1000;

		// Populate eigen matrix and vector to use the eigen solver
		Eigen::MatrixXf eigen_M = Eigen::MatrixXf(dimensions, dimensions);
		Eigen::VectorXf eigen_rhs = Eigen::VectorXf(dimensions);

		for (int l = 0; l < M.size(); l++) {
			for (int k = 0; k < M[0].size(); k++) {
				eigen_M(l, k) = M[l][k];
			}
		}

		for (int k = 0; k < rhs.size(); k++) {
			eigen_rhs[k] = rhs[k];
		}

		// Solve for lambda in M*dv = rhs linear problem using conjugate gradient algorithm
		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
		cg.compute(eigen_M);
		Eigen::VectorXf eigen_dv = cg.solve(eigen_rhs);

		// Provided linear solver does not provide correct solutions for some reason.
		// ConjGrad(dimensions, &M_I, &dv[0], &rhs[0], 0.000000001f, &max_steps);

		// Update velocities and positions
		for (int i = 0; i < dimensions; i += 3) {
			dx[i] = (pVector[i/3]->m_Velocity[0] + eigen_dv[i])*dt;
			dx[i + 1] = (pVector[i/3]->m_Velocity[1] + eigen_dv[i + 1])*dt;
		}
		for (int i = 0; i < pVector.size(); i++) {
			pVector[i]->m_Position += Vec3f(dx[i*3], dx[i*3 + 1], 0);
			pVector[i]->m_Velocity += Vec3f(eigen_dv[i*3], eigen_dv[i*3 + 1], 0);
		}

	} else {
		// loop through all the particles
		for (auto &i : pVector) {
			Vec3f initPos = i->m_Position;
			Vec3f initVel = i->m_Velocity;
			Vec3f initAcc = i->m_Force/i->m_Mass;

			Vec3f newVelocity = initVel + initAcc*dt;
			Vec3f newPosition = initPos + initVel*dt;

			if (type==semi) {
				newPosition = initPos + newVelocity*dt;
			}

			i->m_Position = newPosition;
			i->m_Velocity = newVelocity;
		}
	}
}
