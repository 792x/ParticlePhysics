#include "Particle.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../GL/glut.h"
#endif
#include <string>
using namespace std;



Particle::Particle(const Vec3f &constructPos, float mass, int index, bool drawable) :
	m_ConstructPos(constructPos),
	m_Position(constructPos),
	m_OldPosition(constructPos),
	m_Velocity(Vec3f(0.0, 0.0, 0.0)),
	m_Force(Vec3f(0.0, 0.0, 0.0)),
	m_Index(index),
	m_Mass(mass),
	drawable(drawable){}

Particle::Particle() {

}

void Particle::undo_next_state() {
	m_OldPosition = m_OldOldPosition;
	m_OldVelocity = m_OldOldVelocity;
	m_Position = m_OldPosition;
	m_Velocity = m_OldVelocity;
}

void Particle::next_state(Vec3f new_pos, Vec3f new_vel) {
	m_OldOldPosition = m_OldPosition;
	m_OldOldVelocity = m_OldVelocity;
	m_OldPosition = m_Position;
	m_OldVelocity = m_Velocity;
	m_Position = new_pos;
	m_Velocity = new_vel;
}

void Particle::reset() {
	m_Position = m_ConstructPos;
	m_OldPosition = m_ConstructPos;
	m_Velocity = Vec3f(0.0, 0.0, 0.0);
	m_Force = Vec3f(0.0, 0.0, 0.0);
}
void Particle::draw() {
	if (!drawable) return;
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] - h/2.0, m_Position[1] - h/2.0);
	glVertex2f(m_Position[0] + h/2.0, m_Position[1] - h/2.0);
	glVertex2f(m_Position[0] + h/2.0, m_Position[1] + h/2.0);
	glVertex2f(m_Position[0] - h/2.0, m_Position[1] + h/2.0);
	glEnd();
}

string Particle::getType()
{
	return string("PARTICLE");
}

void Particle::setDrawable(bool d)
{
	this->drawable = d;
}
