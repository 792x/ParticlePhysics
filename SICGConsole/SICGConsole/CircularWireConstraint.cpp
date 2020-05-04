#include "CircularWireConstraint.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec3f &vect, float radius) {
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < 360; i = i + 18) {
		float degInRad = i*PI/180;
		glVertex2f(vect[0] + cos(degInRad)*radius, vect[1] + sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec3f &center, const float radius)
	: Constraint({p}), m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw() {
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::m_C() {
	Vec3f diff = m_p->m_Position - m_center;
	return diff*diff - m_radius*m_radius;
}

float CircularWireConstraint::m_Cd() {
	Vec3f p = (m_p->m_Position - m_center);
	Vec3f v = m_p->m_Velocity;

	return 2*p*v;
}

std::vector<Vec3f> CircularWireConstraint::m_j() {
	std::vector<Vec3f> j;
	j.push_back((m_p->m_Position - m_center)*2);
	return j;
}

std::vector<Vec3f> CircularWireConstraint::m_jd() {
	std::vector<Vec3f> jd;
	jd.push_back(m_p->m_Velocity*2);
	return jd;
}