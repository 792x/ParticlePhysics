#include "ConstraintSolver.h"
#include "linearSolver.h"
#include <Eigen/IterativeLinearSolvers>

std::vector<std::vector<float>> ConstraintSolver::multiply_matrix_by_matrix(std::vector<std::vector<
	float>> v1,
																			std::vector<std::vector<
																				float>> v2) {
	std::vector<std::vector<float>>
		result = std::vector<std::vector<float>>(v1.size(), std::vector<float>(v2[0].size()));
	for (int i = 0; i < v1.size(); i++) {
		for (int j = 0; j < v2[0].size(); j++) {
			result[i][j] = 0;
			for (int k = 0; k < v1[0].size(); k++) {
				result[i][j] = result[i][j] + v1[i][k]*v2[k][j];
			}
		}
	}
	return result;
}

std::vector<float> ConstraintSolver::multiply_matrix_by_vector(std::vector<std::vector<float>> v1,
															   std::vector<float> v2) {
	std::vector<float> result = std::vector<float>(v1.size());
	for (int i = 0; i < v1.size(); i++) {
		result[i] = 0;
		for (int j = 0; j < v2.size(); j++) {
			result[i] = result[i] + v1[i][j]*v2[j];
		}
	}
	return result;
}

std::vector<float> ConstraintSolver::multiply_vector_by_scalar(std::vector<float> v,
															   float s) {
	std::vector<float> result = std::vector<float>(v.size());
	for (int i = 0; i < v.size(); i++) {
		result[i] = v[i]*s;
	}
	return result;
}

std::vector<float> ConstraintSolver::subtract_vector_from_vector(std::vector<float> v1,
																 std::vector<float> v2) {
	std::vector<float> result = std::vector<float>(v1.size());
	for (int i = 0; i < v1.size(); i++) {
		result[i] = v1[i] - v2[i];
	}
	return result;
}

std::vector<std::vector<float>> ConstraintSolver::multiply_matrix_by_scalar(std::vector<std::vector<
	float>> v, float s) {
	std::vector<std::vector<float>>
		result = std::vector<std::vector<float>>(v.size(), std::vector<float>(v[0].size()));
	for (int i = 0; i < v.size(); i++) {
		for (int j = 0; j < v[0].size(); j++) {
			result[i][j] = v[i][j]*s;
		}
	}
	return result;
}

std::vector<std::vector<float>> ConstraintSolver::subtract_matrix_from_matrix(std::vector<std::vector<
	float>> v1, std::vector<std::vector<
	float>> v2) {
	std::vector<std::vector<float>>
		result = std::vector<std::vector<float>>(v1.size(), std::vector<float>(v2[0].size()));
	for (int i = 0; i < v1.size(); i++) {
		for (int j = 0; j < v2[0].size(); j++) {
			for (int k = 0; k < v1[0].size(); k++) {
				result[i][j] = v1[i][k] - v2[k][j];
			}
		}
	}
	return result;
}

std::vector<std::vector<float>> ConstraintSolver::add_matrix_to_matrix(std::vector<std::vector<
	float>> v1, std::vector<std::vector<
	float>> v2) {
	std::vector<std::vector<float>>
		result = std::vector<std::vector<float>>(v1.size(), std::vector<float>(v2[0].size()));
	for (int i = 0; i < v1.size(); i++) {
		for (int j = 0; j < v2[0].size(); j++) {
			for (int k = 0; k < v1[0].size(); k++) {
				result[i][j] = v1[i][k] + v2[k][j];
			}
		}
	}
	return result;
}

void ConstraintSolver::solve(std::vector<Particle *> pVector,
							 std::vector<Constraint *> cVector,
							 float Ks,
							 float Kd) {
	if (!cVector.empty()) {
		int dimensions = 3;
		// Velocity vector
		std::vector<float> q_dot = std::vector<float>(pVector.size()*3, 0);
		// Force vector
		std::vector<float> Q = std::vector<float>(pVector.size()*3, 0);
		// Constraint vector
		std::vector<float> C = std::vector<float>(cVector.size(), 0);
		// Jacobian of C * q
		std::vector<float> C_dot = std::vector<float>(cVector.size(), 0);

		// Mass matrix
		std::vector<std::vector<float>> M =
			std::vector<std::vector<float>>(pVector.size()*3,
											std::vector<float>(pVector.size()*3, 0));
		// Inverse mass matrix
		std::vector<std::vector<float>> W =
			std::vector<std::vector<float>>(pVector.size()*3,
											std::vector<float>(pVector.size()*3, 0));
		// Jacobian of constraint vector
		std::vector<std::vector<float>> J =
			std::vector<std::vector<float>>(cVector.size(),
											std::vector<float>(pVector.size()*3, 0));
		// Transposition of J
		std::vector<std::vector<float>> J_T =
			std::vector<std::vector<float>>(pVector.size()*3,
											std::vector<float>(cVector.size(), 0));
		// Time derivative of the Jacobian
		std::vector<std::vector<float>> J_dot =
			std::vector<std::vector<float>>(cVector.size(),
											std::vector<float>(pVector.size()*3, 0));

		// Fill matrices and vectors for all particles
		for (int i = 0; i < pVector.size()*dimensions; i += dimensions) {
			for (int j = 0; j < 3; j++) {
				M[i + j][i + j] = pVector[i/dimensions]->m_Mass;
				W[i + j][i + j] = 1/pVector[i/dimensions]->m_Mass;
				Q[i + j] = pVector[i/dimensions]->m_Force[j];
				q_dot[i + j] = pVector[i/dimensions]->m_Velocity[j];
			}
		}

		// Fill matrices and vectors for all constraints
		for (int i = 0; i < cVector.size(); i++) {
			C[i] = cVector[i]->m_C();
			C_dot[i] = cVector[i]->m_C_dot();
			std::vector<Vec3f> j = cVector[i]->m_j();
			std::vector<Vec3f> j_dot = cVector[i]->m_j_dot();
			std::vector<Particle *> constraint_particles = cVector[i]->get_particles();
			for (int k = 0; k < constraint_particles.size(); k++) {
				for (int l = 0; l < dimensions; l++) {
					J_dot[i][constraint_particles[k]->m_Index * dimensions + l] = j_dot[k][l];
					J[i][constraint_particles[k]->m_Index * dimensions + l] = j[k][l];
					J_T[constraint_particles[k]->m_Index * dimensions + l][i] = j[k][l];
				}
			}
		}

		// Things we are able to evaluate from acquired matrices and vectors
		std::vector<std::vector<float>> JW = multiply_matrix_by_matrix(J, W);
		std::vector<float> J_dot_q_dot = multiply_matrix_by_vector(J_dot, q_dot);
		std::vector<float> KsC = multiply_vector_by_scalar(C, Ks);
		std::vector<float> KdC_dot = multiply_vector_by_scalar(C_dot, Kd);
		std::vector<float> JWQ = multiply_matrix_by_vector(JW, Q);
		std::vector<std::vector<float>> JWJ_T = multiply_matrix_by_matrix(JW, J_T);

		// Only used for provided linear solver
		// implicitMatrix *JWJ_T_I = new implicitMatrix(&JWJ_T);

		// Make J_dot_q_dot negative
		J_dot_q_dot = multiply_vector_by_scalar(J_dot_q_dot, -1.0f);

		// Right hand side of final constraint force equation with feedback by substracting everything
		std::vector<float> rhs = subtract_vector_from_vector(J_dot_q_dot, JWQ);
		rhs = subtract_vector_from_vector(rhs, KsC);
		rhs = subtract_vector_from_vector(rhs, KdC_dot);

		std::vector<double> lambda(cVector.size(), 0);
		std::vector<double> rhs_double(cVector.size(), 0);

		// Go from float to double
		for (int i = 0; i < rhs.size(); i++) {
			rhs_double[i] = rhs[i];
		}

		// Populate eigen matrix and vector to use the eigen solver
		Eigen::MatrixXf eigen_JWJ_T = Eigen::MatrixXf(cVector.size(), cVector.size());
		Eigen::VectorXf eigen_rhs = Eigen::VectorXf(cVector.size());
		for (int l = 0; l < JWJ_T.size(); l++) {
			for (int k = 0; k < JWJ_T[0].size(); k++) {
				eigen_JWJ_T(l, k) = JWJ_T[l][k];
			}
		}

		for (int k = 0; k < cVector.size(); k++) {
			eigen_rhs[k] = rhs[k];
		}

		// Solve for lambda in JWJ_T*lambda = rhs linear problem using conjugate gradient algorithm
		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
		cg.compute(eigen_JWJ_T);
		Eigen::VectorXf eigen_lambda = cg.solve(eigen_rhs);

		// Provided linear solver does not provide correct solutions for some reason.
		// int max_steps = 1000;
		// ConjGrad(cVector.size(), JWJ_T_I, &lambda[0], &rhs_double[0], 0.000000001f, &max_steps);

		std::vector<float> lambdaf = std::vector<float>(cVector.size(), 0);

		// Go from double to float
		for (int i = 0; i < cVector.size(); i++) {
			lambdaf[i] = eigen_lambda[i];
		}

		// Solve for constraint force
		std::vector<float> Q_hat = multiply_matrix_by_vector(J_T, lambdaf);

		// Apply constraint force
		for (int i = 0; i < pVector.size(); i++) {
			for (int j = 0; j < dimensions; j++) {
				pVector[i]->m_Force[j] += Q_hat[i*dimensions + j];
			}
		}
	}
}
