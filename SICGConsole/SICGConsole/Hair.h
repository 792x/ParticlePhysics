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
		Vec3f center = { 0,0,0 }, float mass=0.1, float radius=0.2);
	void draw() override;
private:
	void addForces(vector<Force*>& fVector);
	void addConstraints(vector<Constraint*>& cVector);
	float mass = 1.0f, radius = 0.2, kd = 30, ks = 80, angle=140.0/180 * 2*PI;
	//float mass = 1.0f, radius = 0.2, kd = 15.0, ks = 150.0, angle=140.0/180 * PI;
	float ang_kd = 0.4, ang_ks = 0.4; // angular spring constant
	Vec3f offset_l{ -0.01,-0.03,0 }; //offset of hair particles
	Vec3f offset_r{ 0.01,-0.03,0 };
	float hair_particle_dist = 0.01;
	int particles_per_hair = 10;
	Vec3f center;

	vector<float> angles  = { 45.0 / 180 * PI, 35.0 / 180 * PI, 55.0 / 180 * PI };
	

};
