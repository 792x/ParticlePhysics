#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
  public:
	RodConstraint(Particle *p1, Particle *p2, float dist);

	void draw() override;
	float m_C() override;
	float m_C_dot() override;
	std::vector<Vec3f> m_j() override;
	std::vector<Vec3f> m_j_dot() override;

  private:
	Particle *const m_p1;
	Particle *const m_p2;
	float const m_dist;
};
