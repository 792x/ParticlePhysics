#pragma once

#include "Force.h"
#include <utility>
#include <vector>

class Constraint {
  public:
	explicit Constraint(std::vector<Particle *> particles) : particles(std::move(particles)) {}
	virtual ~Constraint() = default;

	virtual void draw() = 0;
	virtual float m_C() = 0;
	virtual float m_Cd() = 0;
	virtual std::vector<Vec3f> m_j() = 0;
	virtual std::vector<Vec3f> m_jd() = 0;

	std::vector<Particle*> get_particles() {
		return particles;
	};

  protected:
	std::vector<Particle *> particles;
};


