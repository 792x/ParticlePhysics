#pragma once
#include "Particle.h"
#include <vector>

// Abstract class
class Force {
  public:
	virtual ~Force() = default;

	virtual void target(std::vector<Particle *> particles) = 0;
	virtual void draw() = 0;
	virtual void apply() = 0;
	std::vector<Particle *> particles;
};