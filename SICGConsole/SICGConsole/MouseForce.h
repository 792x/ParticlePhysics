#pragma once

#include <vector>
#include "Particle.h"
#include "Force.h"

class MouseForce : public Force {
  public:
	MouseForce(Particle *p, Vec3f &mouse, double ks, double kd);

	void set_mouse(const Vec3f &mouse);
	void target(std::vector<Particle *> particles) override;
	void draw() override;
	void apply() override;
  private:
	Particle * const m_p;
	Vec3f m_mloc;
	double const m_ks, m_kd; // spring strength constants
};
