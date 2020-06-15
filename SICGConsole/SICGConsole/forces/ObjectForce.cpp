#include "ObjectForce.h"

ObjectForce::ObjectForce(std::vector<Particle*> pVector, std::vector<SolidObject*> soVector)
{
	particles = pVector;
	objects = soVector;
}

// Set the target of the object force.
void ObjectForce::target(std::vector<Particle*> particles)
{
	for (int i = 0; i < particles.size(); i++) {
		this->particles.push_back(particles[i]);
	}
}

// Apply the object force.
void ObjectForce::apply()
{
	for (int i = 0; i < objects.size();i++) {
		for (int j = i + 1; j < objects.size(); j++) {
			if (objects[i]->ObjectsCollide(objects[j])) {
				objects[i]->computeForce(objects[j]);
				objects[i]->computeForceObject(objects[j]);
			}
		}
	}

	for (int i = 0; i < particles.size(); i++) {
		float x1l = particles[i]->m_Position[0];
		float x1r = particles[i]->m_Position[0] + 0.01;
		float y1l = particles[i]->m_Position[1];
		float y1u = particles[i]->m_Position[1] + 0.01;

		for (int j = i + 1; j < particles.size(); j++) {
			float x2l = particles[j]->m_Position[0];
			float x2r = particles[j]->m_Position[0] + 0.01;
			float y2l = particles[j]->m_Position[1];
			float y2u = particles[j]->m_Position[1] + 0.01;

			if (!(x1l >= x2r || x2l >= x1r)) {
				if (!(y1l >= y2u || y2l >= y1u)) {
					Vec3f tempVel = particles[i]->m_Velocity;
					Vec3f forceDiff = particles[i]->m_Force + particles[j]->m_Force;
					particles[i]->m_Force += forceDiff;
					particles[j]->m_Force += forceDiff;
					particles[i]->m_Velocity = (particles[i]->m_Mass - particles[j]->m_Mass) / (particles[i]->m_Mass + particles[j]->m_Mass) * particles[i]->m_Velocity + (2 * particles[j]->m_Mass) / (particles[i]->m_Mass + particles[j]->m_Mass) * particles[j]->m_Velocity;
					particles[j]->m_Velocity = (particles[j]->m_Mass - particles[i]->m_Mass) / (particles[i]->m_Mass + particles[j]->m_Mass) * particles[j]->m_Velocity + (2 * particles[i]->m_Mass) / (particles[i]->m_Mass + particles[j]->m_Mass) * tempVel;
				};
			};
		}
	}
}

void ObjectForce::draw()
{
}
