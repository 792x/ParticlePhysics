#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
  public:
	CircularWireConstraint(Particle *p, const Vec3f &center, const float radius);

	void draw() override;

	float m_C() override;
	float m_Cd() override;
	std::vector<Vec3f> m_j() override;
	std::vector<Vec3f> m_jd() override;

  private:
	Particle *const m_p;
	Vec3f const m_center;
	double const m_radius;
};
