#pragma once
#include "../forces/Force.h"
#include "../objects/SolidObject.h"
#include "../objects/Cloth.h"

class ObjectForce :
	public Force
{
public:
	ObjectForce(std::vector<Particle*> pVector, std::vector<SolidObject*> soVector);
	void apply() override;
	void draw() override;
	void target(std::vector<Particle*> particles) override;
	std::vector<SolidObject*> objects;
	std::vector<Particle*> particles;
	Vec3f contactPoint;
private:
	Vec3f collision_force;
};