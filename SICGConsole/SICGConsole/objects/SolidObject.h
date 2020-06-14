#pragma once
#include "Object.h"
#include <Eigen/Dense>
#ifdef __APPLE__

#include "../include/linux/gfx/vec3.h"
#include "../include/linux/gfx/mat3.h"

#else
#include "..\include\gfx\vec3.h"
#include "..\include\gfx\mat3.h"
#endif
#include <vector>

using namespace Eigen;
using namespace std;
class SolidObject :
	public Particle
{
public:
	SolidObject(int x, int y, Vec3f bottom_left_pos, std::string type = "SOLIDOBJECT", float p_mass = 1.0, float dist = 0.05);
	void state_to_array(float* y);
	void array_to_state(float* y);
	void computeR(Vec3f rotationCenter, float angle);
	void ddt_Calculations();
	void draw() override;
	void init();
	bool object_selected(Vec2f mouse);
	void set_new_position(Vec3f mouse);
	void computeForce(Particle* p);
	void computeForceObject(SolidObject* so);
	void computeTorque();

	bool is_collid(Particle*);
	bool ObjectsCollide(SolidObject*);
	string getType() override;
	string type;

	int xn, yn; //width and high
	Vec3f bot_left_pos; // bottom left position i.e. origin
	float p_mass; //practicle mass
	float dist = 0.5; // distance between each particles
																																		  //constat
	Matrix3f Ibody, Ibodyinv;

	//state varaibles
	//Vec3f x; //position of ridgid body
	Vec3f P, L;
	Matrix3f R;

	//Dervied quantities
	Matrix3f Iinv;
	Vec3f v, omega; //v(t), w(t)

	//computed quantities
	Vec3f force, torque;

	//object points
	Vec3f top_left;// top left
	Vec3f top_right; // top right 
	Vec3f bottom_right;// bottom right
	Vec3f bottom_left;// bottom left

};

