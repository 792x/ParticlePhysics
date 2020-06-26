#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../stdafx.h"
#include "../GL/glut.h"
#endif

#include <utility>
#include "MouseForce.h"
#include "../solvers/ConstraintSolver.h"

MouseForce::MouseForce(Particle* p, Vec3f& mouse, double ks, double kd) : m_p(p), m_mloc(mouse), m_ks(ks), m_kd(kd) {}

void MouseForce::set_mouse(const Vec3f& mouse) {
	m_mloc = mouse;
}

void MouseForce::target(std::vector<Particle*> particles) {

}

// Apply the mouse force.
void MouseForce::apply() {
	Vec3f x = (m_p->m_Position - m_mloc);
	Vec3f v = (m_p->m_Velocity);
	float dist = (sqrt(x[0] * x[0] + x[1] * x[1]));

	// check the distance from the particle before applying a force, this avoids some minor kickbacks/instabilities for explicit solvers
	if (dist > 0.003 * 3) {
		Vec3f f = (m_ks * (norm(x)) + m_kd * ((v * x) / norm(x))) * (x / norm(x));
		m_p->m_Force -= f;
	}
}

void MouseForce::draw() {
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
	glColor3f(1, 0, 0);
	glVertex2f(m_mloc[0], m_mloc[1]);
	glEnd();
}