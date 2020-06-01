#pragma once

#include <vector>
#include "Particle.h"
#include "Force.h"

class GravityForce : public Force {
  public:
	explicit GravityForce(std::vector<Particle *> particles);
	void target(std::vector<Particle *> particles) override;
	void apply() override;
	void draw() override;
	std::vector<std::vector<float>> jacobian() override;
  private:
	static const Vec3f standard_gravity;
};
