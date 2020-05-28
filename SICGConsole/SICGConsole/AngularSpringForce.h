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

	void set_dist(float dist) { Force::dist = dist; };
	void set_ks(float ks) { Force::ks = ks; };
	void set_kd(float kd) { Force::kd = kd; };

	//float dist = Force::dist;
	//float kd = Force::kd;
	//float ks = Force::ks;
	
	float const dist, ks, kd; // rest length, spring constants
};