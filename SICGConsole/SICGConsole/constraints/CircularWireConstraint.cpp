#include "CircularWireConstraint.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../GL/glut.h"
#endif

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec3f& vect, float radius) {
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < 360; i = i + 18) {
		float degInRad = i * PI / 180;
		glVertex2f(vect[0] + cos(degInRad) * radius, vect[1] + sin(degInRad) * radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle* p, const Vec3f& center, const float radius)
	: Constraint({ p }), m_p(p), m_center(center), m_radius(radius) {}

// Function to draw the CircularWireConstraint.
void CircularWireConstraint::draw() {
	draw_circle(m_center, m_radius);
}

// Function that computes m_C of the CircularWireConstraint.
float CircularWireConstraint::m_C() {
	Vec3f diff = m_p->m_Position - m_center;
	Vec3f v = m_p->m_Velocity;
	float m_C = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2] - m_radius * m_radius;
	return m_C;
}

// Function that computes m_C_dot of the CircularWireConstraint.
float CircularWireConstraint::m_C_dot() {
	Vec3f diff = m_p->m_Position - m_center;
	Vec3f v = m_p->m_Velocity;
	float m_C_dot = 2.f * diff * v;
	return m_C_dot;
}

// Function that computes m_j of the CircularWireConstraint.
std::vector<Vec3f> CircularWireConstraint::m_j() {
	std::vector<Vec3f> j;
	Vec3f diff = m_p->m_Position - m_center;
	Vec3f v = m_p->m_Velocity;
	j.emplace_back(Vec3f(2 * diff[0], 2 * diff[1], 2 * diff[2]));
	return j;
}

// Function that computes m_j_dot of the CircularWireConstraint.
std::vector<Vec3f> CircularWireConstraint::m_j_dot() {
	std::vector<Vec3f> j_dot;
	Vec3f v = m_p->m_Velocity;
	j_dot.emplace_back(Vec3f(2 * v[0], 2 * v[1], 2 * v[2]));
	return j_dot;
}