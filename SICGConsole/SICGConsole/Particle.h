#pragma once

#ifdef __APPLE__

#include "include/linux/gfx/vec3.h"

#else

#include "include\gfx\vec3.h"
#endif

class Particle {
  public:
	Particle(const Vec3f &constructPos, float mass, int index);
	virtual ~Particle() = default;

	void reset();
	void draw();

	Vec3f m_ConstructPos;
	Vec3f m_OldPosition; // used for the Verlet integration scheme
	Vec3f m_Position;
	Vec3f m_Velocity;
	Vec3f m_Force;
	int m_Index;
	float m_Mass;
};
