#pragma once

#include "../objects/Particle.h"
#include "Force.h"

class SpringForce : public Force {
  public:
	SpringForce(Particle *p1, Particle *p2, float dist, float ks, float kd);
	SpringForce(std::vector<Particle*> particles, float dist, float ks, float kd);

	void deformation_update();
	bool deformation_check();
	void target(std::vector<Particle *> particles) override;
	void reset();
	void draw() override;
	void apply() override;
	std::vector<std::vector<float>> jacobian();

  private:
	float const m_dist;     // rest length
	float const m_ks, m_kd; // spring strength constants
	float p_len; // length of the spring in the previous step
	float c_len; // length of the spring in the current step
};
