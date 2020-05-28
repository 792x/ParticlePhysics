#pragma once

#include "Particle.h"
#include "Force.h"

class SpringForce : public Force {
  public:
	SpringForce(Particle *p1, Particle *p2, float dist, float ks, float kd);
	SpringForce(std::vector<Particle*> particles, float dist, float ks, float kd);

	void target(std::vector<Particle *> particles) override;
	void draw() override;
	void apply() override;
	void set_dist(float dist) { Force::dist = dist; };
	void set_ks(float ks) { Force::ks = ks; };
	void set_kd(float kd) { Force::kd = kd; };

	//float dist = Force::dist;
	//float kd = Force::kd;
	//float ks = Force::ks;
	float const dist;     // rest length
	float const ks, kd; // spring strength constants
};
