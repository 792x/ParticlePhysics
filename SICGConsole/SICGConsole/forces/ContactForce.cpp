#include "ContactForce.h"

ContactForce::ContactForce(Cloth* cloth, SolidObject* so) :solid_object(so)
{
	target(cloth->particles);
}

// Set the contact force target.
void ContactForce::target(std::vector<Particle*> particles)
{
	for (int i = 0; i < particles.size(); i++) {
		this->particles.push_back(particles[i]);
	}
}

// Apply the contact force.
void ContactForce::apply()
{
	float df = 4;
	for (auto p : particles) {
		if (solid_object->is_collid(p)) {
			p->m_Force += df * solid_object->m_Velocity * solid_object->m_Mass / p->m_Mass;
			solid_object->computeForce(p);
		}
	}
}

void ContactForce::draw()
{
}