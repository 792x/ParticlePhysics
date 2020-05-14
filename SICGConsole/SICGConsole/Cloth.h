#pragma once

#ifdef __APPLE__

#include "include/linux/gfx/vec3.h"

#else
#include "include\gfx\vec3.h"
#endif

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

#include<vector>

using namespace std;
class Cloth
{
public:
	// create cloth particles and add to pVector
	Cloth(int x, int y, Vec3f bottom_left_pos, vector<Particle*> &ps, 
		vector<Force*>& fs, vector<Constraint*> &cs, float mass=4.0, float dist = 0.5,
		float ks=0.45, float kd=0.48);
	
	// cloth particles will be created add to ps
	void init(vector<Particle*>& ps, vector<Force*> &fs, vector<Constraint*>& cs, float mass);
	void addForces(vector<Particle*> ps, vector<Force*>& fs);
	void addConstraints(vector<Particle*> ps, vector<Constraint*>& cs);
	void draw();
	~Cloth();
private:
	vector<Particle*> particles;
	int xn, yn; //width and high
	Vec3f bot_left_pos;
	float dist = 0.5; // distance between each particle
	float c_ks; //particle spring force parameter
	float c_kd;
};
