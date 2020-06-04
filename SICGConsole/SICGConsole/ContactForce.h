#pragma once
#include "Force.h"
#include "SolidObject.h"
#include "Cloth.h"
class ContactForce :
	public Force
{
public:
	ContactForce(Cloth* cloth, SolidObject* so);
	void target(std::vector<Particle *> particles) override;
	void apply() override;
	void draw() override;
	SolidObject* solid_object;
private:
	Vec3f collision_force;
};

