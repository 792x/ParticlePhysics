#pragma once

#ifdef __APPLE__

#include "../include/linux/gfx/vec3.h"

#else

#include "..\include\gfx\vec3.h"
#endif

class Particle {
  public:
	Particle(const Vec3f &constructPos, float mass, int index, bool drawable=1, Vec3f col = Vec3f(1.f, 1.f, 1.f));
	Particle();
	virtual ~Particle() = default;

	void reset();
	virtual void draw();
	virtual std::string getType();
	void setDrawable(bool d);
	void undo_next_state();
	void next_state(Vec3f new_pos, Vec3f new_vel);

	Vec3f m_ConstructPos;
	Vec3f m_OldPosition; // used for the Verlet integration scheme
	Vec3f m_Position;
	Vec3f m_Velocity;
	Vec3f m_Force;
	int m_Index;
	float m_Mass;
	bool drawable = 1;

	Vec3f m_OldOldPosition;
	Vec3f m_OldOldVelocity;
	Vec3f m_OldVelocity;

	Vec3f col;
};
