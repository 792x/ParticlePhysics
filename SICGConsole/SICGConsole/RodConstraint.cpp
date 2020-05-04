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
	return diff*diff - m_dist*m_dist;
}

float RodConstraint::m_Cd() {
	Vec3f pDiff = (m_p1->m_Position - m_p2->m_Position)*2;
	Vec3f vDiff = (m_p1->m_Velocity - m_p2->m_Velocity)*2;
	return pDiff*vDiff;
}

std::vector<Vec3f> RodConstraint::m_j() {
	std::vector<Vec3f> j;
	j.push_back((m_p1->m_Position - m_p2->m_Position)*2);
	j.push_back((m_p2->m_Position - m_p1->m_Position)*2);
	return j;
}

std::vector<Vec3f> RodConstraint::m_jd() {
	std::vector<Vec3f> jd;
	jd.push_back((m_p1->m_Velocity - m_p2->m_Velocity)*2);
	jd.push_back((m_p2->m_Velocity - m_p1->m_Velocity)*2);
	return jd;
}

void RodConstraint::draw() {
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
	glEnd();
}
