#include "RodConstraint.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

RodConstraint::RodConstraint(Particle *p1, Particle *p2, float dist) :
	Constraint({p1, p2}), m_p1(p1), m_p2(p2), m_dist(dist) {}

float RodConstraint::m_C() {
	Vec3f diff = m_p1->m_Position - m_p2->m_Position;
	float m_C = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] - m_dist*m_dist;
	return m_C;
}

float RodConstraint::m_C_dot() {
	Vec3f diff = m_p1->m_Position - m_p2->m_Position;
	Vec3f v =m_p1->m_Velocity - m_p2->m_Velocity;
	float m_C_dot = 2.f*diff*v;
	return m_C_dot;
}

std::vector<Vec3f> RodConstraint::m_j() {
	std::vector<Vec3f> j;
	j.push_back((m_p1->m_Position - m_p2->m_Position)*2.f);
	j.push_back((m_p2->m_Position - m_p1->m_Position)*2.f);
	return j;
}

std::vector<Vec3f> RodConstraint::m_j_dot() {
	std::vector<Vec3f> j_dot;
	j_dot.push_back((m_p1->m_Velocity - m_p2->m_Velocity)*2.f);
	j_dot.push_back((m_p2->m_Velocity - m_p1->m_Velocity)*2.f);
	return j_dot;
}

void RodConstraint::draw() {
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
	glEnd();
}
