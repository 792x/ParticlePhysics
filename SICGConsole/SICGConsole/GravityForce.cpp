#include "GravityForce.h"
#include "GL/glut.h"

GravityForce::GravityForce(vector<Particle*> ps):particles(ps)
{
}

void GravityForce::draw()
{
}

void GravityForce::apply()
{
	for (p : particles) {
		p->m_Force += p->mass * g;
	}
}
