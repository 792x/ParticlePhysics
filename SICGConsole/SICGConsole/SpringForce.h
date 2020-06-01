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
	std::vector<std::vector<float>> jacobian() override;

  private:
	float const m_dist;     // rest length
	float const m_ks, m_kd; // spring strength constants
};
