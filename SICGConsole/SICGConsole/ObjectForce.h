#pragma once
#include "../forces/Force.h"
#include "../objects/SolidObject.h"
#include "../objects/Cloth.h"
class ObjectForce :
	public Force
{
public:
	ObjectForce(std::vector<SolidObject*> soVector);
	void apply() override;
	void draw() override;
	void target(std::vector<Particle*> particles) override;
	std::vector<SolidObject*> objects;
private:
	Vec3f collision_force;
};

