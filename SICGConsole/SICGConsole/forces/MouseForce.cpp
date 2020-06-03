#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../stdafx.h"
#include "../GL/glut.h"
#endif

#include <utility>
#include "MouseForce.h"

MouseForce::MouseForce(Particle *p, Vec3f &mouse, double ks, double kd) : m_p(p), m_mloc(mouse), m_ks(ks), m_kd(kd) {}

void MouseForce::set_mouse(const Vec3f &mouse) {
	m_mloc = mouse;
}

void MouseForce::target(std::vector<Particle *> particles) {

}

void MouseForce::apply() {

	Vec3f x = (m_p->m_Position - m_mloc);
	Vec3f v = (m_p->m_Velocity);
	float dist = (sqrt(x[0] * x[0] + x[1] * x[1]));

	Vec3f f = x/norm(x);
	f *= (m_ks*(norm(x) - dist) + m_kd*((x*v)/norm(x)));

	m_p->m_Velocity -= f;
}

void MouseForce::draw() {
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
	glColor3f(1, 0, 0);
	glVertex2f(m_mloc[0], m_mloc[1]);
	glEnd();
}