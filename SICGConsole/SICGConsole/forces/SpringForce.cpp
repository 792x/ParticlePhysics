#include "SpringForce.h"
#include "solvers/ConstraintSolver.h"

#ifdef __APPLE__
#include <GLUT/glut.h>

#include <utility>
#else
#include "../GL/glut.h"
#endif

SpringForce::SpringForce(Particle *p1, Particle * p2, float dist, float ks, float kd) :
	SpringForce({p1, p2}, dist, ks, kd) {}

SpringForce::SpringForce(std::vector<Particle*> particles, float dist, float ks, float kd) : m_dist(dist), m_ks(ks), m_kd(kd)
{
	this->target(std::move(particles));
}

float distance(Particle* p0, Particle* p1) {//TODO: is there a standard function for this?
	float x0 = p0->m_Position[0];
	float y0 = p0->m_Position[1];
	float z0 = p0->m_Position[2];
	float x1 = p1->m_Position[0];
	float y1 = p1->m_Position[1];
	float z1 = p1->m_Position[2];
	return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
}

void SpringForce::deformation_update() {
	p_len = c_len;
	c_len = distance(particles[0], particles[1]);
}

bool SpringForce::deformation_check() {
	float t_len = distance(particles[0], particles[1]);
	if (t_len > p_len * 1.10 || t_len < p_len * 0.90) {
		return false;
	} else {
		return true;
	}
}

void SpringForce::reset() {
	p_len = distance(particles[0], particles[1]);
	c_len = distance(particles[0], particles[1]);
}

void SpringForce::target(std::vector<Particle *> particles) {
	this->particles = particles;
}

void SpringForce::draw() {
	glBegin(GL_LINES);
	glColor3f(0.6, 0.7, 0.8);
	glVertex3f(particles[0]->m_Position[0], particles[0]->m_Position[1], particles[0]->m_Position[2]);
	glColor3f(0.6, 0.7, 0.8);
	glVertex3f(particles[1]->m_Position[0], particles[1]->m_Position[1], particles[1]->m_Position[2]);
	glEnd();
}

void SpringForce::apply() {
	Vec3f x = particles[0]->m_Position - particles[1]->m_Position;
	if (norm(x) > 2 * m_dist) {// Spring loses elasticity
		return;
	}
	Vec3f v = particles[0]->m_Velocity - particles[1]->m_Velocity;
	Vec3f f = x/norm(x);
	f *= (m_ks*(norm(x) - m_dist) + m_kd*((x*v)/norm(x)));
//	Vec3f f = -(m_ks * (norm(x) - m_dist) + m_kd * ((x * v) / norm(x))) * (x / norm(x));

	particles[0]->m_Force -= f;
	particles[1]->m_Force += f;
}

std::vector<std::vector<float>> SpringForce::jacobian() {
	Vec3f x = particles[0]->m_Position - particles[1]->m_Position;
	Vec3f f = x/norm(x);

	std::vector<std::vector<float>>
		mf = std::vector<std::vector<float>>(3, std::vector<float>(3, 0));
	std::vector<std::vector<float>>
		I = std::vector<std::vector<float>>(3, std::vector<float>(3, 0));

	mf[0][0] = f[0] * f[0];
	mf[0][1] = f[0] * f[1];
	mf[1][0] = f[1] * f[0];
	mf[1][1] = f[1] * f[1];

	I[0][0] = 1.f;
	I[0][1] = 0.f;
	I[1][0] = 0.f;
	I[1][1] = 1.f;

	//((1.0 - m_dist / norm(x)) * (I - mf) + mf) * -m_ks;
	std::vector<std::vector<float>> jacobian =
		ConstraintSolver::multiply_matrix_by_scalar((ConstraintSolver::add_matrix_to_matrix(
			ConstraintSolver::multiply_matrix_by_scalar((ConstraintSolver::subtract_matrix_from_matrix(
				I,
				mf)), (1.0 - m_dist/norm(x))),
			mf)), -m_ks);
	return jacobian;
}
