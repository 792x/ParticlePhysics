#/*ifdef __APPLE__
#include <GLUT/glut.h>

#include <utility>
#else*/
#include "AngularSpringForce.h"
#include "GL/glut.h"
//#endif

using namespace std;

AngularSpringForce::AngularSpringForce(std::vector<Particle*> ps, float dist, float ks, float kd):
 dist(dist),ks(ks), kd(kd)
{
	this->target(ps);
}

void AngularSpringForce::draw()
{
}

void AngularSpringForce::apply()
{
	Vec3f l1 = particles[0]->m_Position - particles[1]->m_Position;
	Vec3f l2 = particles[1]->m_Position - particles[2]->m_Position;

	//double cosine = (l1 * l2) / (norm(l1) * norm(l2));

	Vec3f l = particles[0]->m_Position - particles[2]->m_Position;
	Vec3f ld = particles[0]->m_Velocity - particles[2]->m_Velocity;

	double b = norm(l1);
	double c = norm(l2);
	//Vec3f f = -(ks * (norm(l) - sqrt(b * b + c * c - 2 * b * c * cos(dist))) 
	//	+ kd * ((l * ld) / norm(l))) * (l / norm(l));
	// Anugalar force try to maintain the dist, which is the angle of l1 and l2
	Vec3f f = -(ks * (norm(l) - sqrt(b * b + c * c - 2 * b * c * cos(dist))) +
		kd * ((l * ld) / norm(l)));
	f = f * l ;
	f /= norm(l);

	particles[0]->m_Force += f;
	particles[2]->m_Force -= f;

}

void AngularSpringForce::target(std::vector<Particle*> particles)
{
	this->particles = particles;
}
