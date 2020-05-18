#include "ConstraintSolver.h"
#include <Eigen/IterativeLinearSolvers>

void ConstraintSolver::solve(std::vector<Particle *> particles, std::vector<Constraint *> constraints, float Ks, float Kd) {

	if (!constraints.empty()){
		int dims = 3;
		unsigned int pSize = particles.size() * dims;
		unsigned int cSize = constraints.size();

		Eigen::VectorXf qd = Eigen::VectorXf::Zero(pSize);
		Eigen::VectorXf Q = Eigen::VectorXf::Zero(pSize);
		Eigen::MatrixXf M = Eigen::MatrixXf::Zero(pSize, pSize);
		Eigen::MatrixXf W = Eigen::MatrixXf::Zero(pSize, pSize);

		for (int i = 0; i < pSize; i += dims) {
			Particle *p = particles[i / dims];
			for (int d = 0; d < dims; d++) {
				M(i + d, i + d) = p->m_Mass;
				W(i + d, i + d) = 1 / p->m_Mass;
				Q[i + d] = p->m_Force[d];
				qd[i + d] = p->m_Velocity[d];
			}
		}


		Eigen::VectorXf C = Eigen::VectorXf::Zero(cSize);
		Eigen::VectorXf Cd = Eigen::VectorXf::Zero(cSize);
		Eigen::MatrixXf J = Eigen::MatrixXf::Zero(cSize, pSize);
		Eigen::MatrixXf Jt = Eigen::MatrixXf::Zero(pSize, cSize);
		Eigen::MatrixXf Jd = Eigen::MatrixXf::Zero(cSize, pSize);

		for (int i = 0; i < cSize; i++) {
			Constraint* c = constraints[i];
			C[i] = c->m_C();
			Cd[i] = c->m_Cd();
			std::vector<Vec3f> j = c->m_j();
			std::vector<Vec3f> jd = c->m_jd();
			std::vector<Particle*> constraint_particles = c->get_particles();

			for (int k = 0; k < constraint_particles.size(); k++) {
				int pIndex = constraint_particles[k]->m_Index * dims;
				for (int d = 0; d < dims; d++) {
					Jd(i, pIndex + d) = jd[k][d];
					J(i, pIndex + d) = j[k][d];
					Jt(pIndex + d, i) = j[k][d];
				}
			}
		}

		Eigen::MatrixXf JW = J * W;
		Eigen::MatrixXf JWJt = JW * Jt;
		Eigen::VectorXf Jdqd = Jd * qd;
		Eigen::VectorXf JWQ = JW * Q;

		Eigen::VectorXf KsC = Ks * C;
		Eigen::VectorXf KdCd = Kd * Cd;

		Eigen::VectorXf rhs = - Jdqd - JWQ - KsC - KdCd;

		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower|Eigen::Upper> cg;

		cg.compute(JWJt);

		Eigen::VectorXf lambda = cg.solve(rhs);
		Eigen::VectorXf Qh = J.transpose() * lambda;

		for (int i = 0; i < particles.size(); i++) {
			Particle *p = particles[i];
			int index = dims * i;
			for (int d = 0; d < dims; d++) {
				p->m_Force[d] += Qh[index + d];
			}
		}
	}
}