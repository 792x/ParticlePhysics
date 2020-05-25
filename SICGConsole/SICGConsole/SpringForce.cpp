#include "SpringForce.h"

#ifdef __APPLE__
#include <GLUT/glut.h>

#include <utility>
#else
#include "GL/glut.h"
#endif

SpringForce::SpringForce(Particle *p1, Particle * p2, float dist, float ks, float kd) :
	SpringForce({p1, p2}, dist, ks, kd) {}

SpringForce::SpringForce(std::vector<Particle*> particles, float dist, float ks, float kd) : m_dist(dist), m_ks(ks), m_kd(kd)
{
	this->target(std::move(particles));
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
