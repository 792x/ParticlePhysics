#include "ConstraintSolver.h"
#include <Eigen/IterativeLinearSolvers>

void ConstraintSolver::solve(std::vector<Particle *> particles, std::vector<Constraint *> constraints, float Ks, float Kd) {

	if (!constraints.empty()){
		int dimensions = 3;

		// Velocity vector
		Eigen::VectorXf q_dot = Eigen::VectorXf::Zero(particles.size() * dimensions);
		// Force vector
		Eigen::VectorXf Q = Eigen::VectorXf::Zero(particles.size() * dimensions);
		// Constraint vector
		Eigen::VectorXf C = Eigen::VectorXf::Zero(constraints.size());
		// Jacobian of C * q
		Eigen::VectorXf C_dot = Eigen::VectorXf::Zero(constraints.size());

		// Mass matrix
		Eigen::MatrixXf M = Eigen::MatrixXf::Zero(particles.size() * dimensions, particles.size() * dimensions);
		// Inverse mass matrix
		Eigen::MatrixXf W = Eigen::MatrixXf::Zero(particles.size() * dimensions, particles.size() * dimensions);
		// Jacobian of constraint vector
		Eigen::MatrixXf J = Eigen::MatrixXf::Zero(constraints.size(), particles.size() * dimensions);
		// Transposition of J
		Eigen::MatrixXf J_T = Eigen::MatrixXf::Zero(particles.size() * dimensions, constraints.size());
		// Time derivative of the Jacobian
		Eigen::MatrixXf J_dot = Eigen::MatrixXf::Zero(constraints.size(), particles.size() * dimensions);


		// Fill matrices and vectors for all particles
		for (int i = 0; i < particles.size() * dimensions; i += dimensions) {
			Particle *p = particles[i / dimensions];
			for (int dim = 0; dim < dimensions; dim++) {
				M(i + dim, i + dim) = p->m_Mass;
				W(i + dim, i + dim) = 1 / p->m_Mass;
				Q[i + dim] = p->m_Force[dim];
				q_dot[i + dim] = p->m_Velocity[dim];
			}
		}

		// Fill matrices and vectors for all constraints
		for (int i = 0; i < constraints.size(); i++) {
			Constraint* c = constraints[i];
			C[i] = c->m_C();
			C_dot[i] = c->m_C_dot();
			std::vector<Vec3f> j = c->m_j();
			std::vector<Vec3f> j_dot = c->m_j_dot();
			std::vector<Particle*> constraint_particles = c->get_particles();

			for (int k = 0; k < constraint_particles.size(); k++) {
				int index = constraint_particles[k]->m_Index * dimensions;
				for (int d = 0; d < dimensions; d++) {
					J_dot(i, index + d) = j_dot[k][d];
					J(i, index + d) = j[k][d];
					J_T(index + d, i) = j[k][d];
				}
			}
		}

		// Things we are able to evaluate from acquired matrices and vectors
		Eigen::MatrixXf JW = J * W;
		Eigen::VectorXf J_dot_q_dot = J_dot * q_dot;
		Eigen::VectorXf KdC_dot = Kd * C_dot;
		Eigen::VectorXf JWQ = JW * Q;
		Eigen::VectorXf KsC = Ks * C;
		Eigen::MatrixXf JWJ_T = JW * J_T;


		// Right hand side of final constraint force equation with feedback
		Eigen::VectorXf rhs = -J_dot_q_dot - JWQ - KsC - KdC_dot;

		// Solve for lambda in JWJ_T*lambda = rhs linear problem using conjugate gradient algorithm
		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower|Eigen::Upper> cg;
		cg.compute(JWJ_T);
		Eigen::VectorXf lambda = cg.solve(rhs);

		// Solve for constraint force
		Eigen::VectorXf Q_hat = J.transpose() * lambda;

		// Apply constraint force
		for (int i = 0; i < particles.size(); i++) {
			Particle *p = particles[i];
			int index = dimensions * i;
			for (int d = 0; d < dimensions; d++) {
				p->m_Force[d] += Q_hat[index + d];
			}
		}
	}
}