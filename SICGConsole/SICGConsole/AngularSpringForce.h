#pragma once

#include <vector>
#include "Particle.h"
#include "Force.h"

class AngularSpringForce : public Force {
public:
	// three-particle vector
	AngularSpringForce(std::vector<Particle*> ps, float dist, float ks, float kd);

	void draw() override;
	void apply() override;
	void target(std::vector<Particle*> particles);
private:
	float const dist, ks, kd; // rest length, spring constants
};