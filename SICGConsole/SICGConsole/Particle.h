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

	void undo_next_state();
	void next_state(Vec3f new_pos, Vec3f new_vel);
	void reset();
	void draw();

	Vec3f m_ConstructPos;
	Vec3f m_OldOldPosition;
	Vec3f m_OldPosition;
	Vec3f m_Position;
	Vec3f m_OldOldVelocity;
	Vec3f m_OldVelocity;
	Vec3f m_Velocity;
	Vec3f m_Force;
	int m_Index;
	float m_Mass;

	
};
