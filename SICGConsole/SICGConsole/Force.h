#pragma once
#include "Particle.h"
#include <vector>
#include "gfx/mat3.h"

// Abstract class
class Force {
  public:
	virtual ~Force() = default;

	virtual void target(std::vector<Particle *> particles) = 0;
	virtual void draw() = 0;
	virtual void apply() = 0;
	std::vector<Particle *> particles;
	void reset() {
		for (Particle* p : particles) {
			p->m_Force = Vec3f(0.0, 0.0, 0.0);
		}
	}
	Mat3 Jx; // Jacobian with respect to position
	Mat3 Jv; // Jacobian with resepct to velocity
	float dist; // rest length
	float ks; // stiffness constant (aka. spring constant)
	float kd; // damping constant
};