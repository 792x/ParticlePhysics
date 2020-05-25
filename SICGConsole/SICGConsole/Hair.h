#pragma once

#ifdef __APPLE__

#include "include/linux/gfx/vec3.h"

#else
#include "include\gfx\vec3.h"
#endif

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
#include "Object.h"

#include<vector>
#define PI 3.1415926535897932384626433832795

using namespace std;
class Hair : public Object {
public:
	Hair(vector<Particle*>& pVector, vector<Force*>& fVector, vector<Constraint*>& cVector,
		Vec3f center = { 0,0,0 }, float mass=0.5, float radius=0.2);
	void draw() override;
private:
	void addForces(vector<Force*>& fVector);
	void addConstraints(vector<Constraint*>& cVector);
	float mass = 0.5f, radius = 0.2, kd = 1.5, ks = 150.0, angle=140.0/180 * PI;
	Vec3f center;
	

};
