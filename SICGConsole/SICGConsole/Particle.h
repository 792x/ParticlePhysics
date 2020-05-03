#pragma once

#ifdef __APPLE__
#include "include/linux/gfx/vec2.h"
#else
#include "include\gfx\vec2.h"
#endif



class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;

	float mass;
};
