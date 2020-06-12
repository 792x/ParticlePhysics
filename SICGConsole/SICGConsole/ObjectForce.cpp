#include "ObjectForce.h"

ObjectForce::ObjectForce(std::vector<SolidObject*> soVector)
{
	objects = soVector;
}

void ObjectForce::target(std::vector<Particle*> particles)
{
	for (int i = 0; i < particles.size(); i++) {
		this->particles.push_back(particles[i]);
	}
}

void ObjectForce::apply()
{
	float df = 4;
	for (int i = 0; i < objects.size();i++) {
		for (int j = i+1; j < objects.size(); j++) {
				if (objects[i]->ObjectsCollide(objects[j])) {
					objects[i]->computeForce(objects[j]);
						//objects[j]->computeForce(objects[i]);
				}
			}
		
	}
}

void ObjectForce::draw()
{
}
