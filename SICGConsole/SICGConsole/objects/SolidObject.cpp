#include "SolidObject.h"
#include <Eigen/Dense>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../GL/glut.h"
#endif

using namespace Eigen;
static Vec3f eigen_to_vec(Vector3f a) {
	return Vec3f(a[0], a[1], a[2]);
}
static Matrix3f star(Vec3f a) {
	Matrix3f m(3, 3);
	m(0, 0) = 0;	 m(0, 1) = -a[2];	m(0, 2) = a[1];

	m(1, 0) = a[2];	 m(1, 1) = 0;	m(1, 2) = -a[0];

	m(2, 0) = -a[1]; m(2, 1) = a[0]; m(2, 2) = 0;

	return m;
}

SolidObject::SolidObject(int x, int y, Vec3f bottom_left_pos,std::string type, float p_mass, float dist):
	xn(x),yn(y),bot_left_pos(bottom_left_pos), type(type) ,p_mass(p_mass),dist(dist)
{
	/*particle implementation of solid object*/
	m_Position = bottom_left_pos;
	/*particle implementation of solid object*/
	init();
}

//void SolidObject::state_to_array(float *y)
//{
//	Vec3f x = m_Position;
//	*y++ = x[0];	*y++ = x[1];	*y++ = x[2];

//	for (int i = 0; i < 3; i++) /* copy rotation matrix */
//		for (int j = 0; j < 3; j++)
//			*y++ = R(i,j);

//	*y++ = P[0];	*y++ = P[1];	*y++ = P[2];

//	*y++ = L[0];	*y++ = L[1];	*y++ = L[2];
//}

//void SolidObject::array_to_state(float* y)
//{
//	Vec3f x = m_Position;
//	x[0] = *y++;	x[1] = *y++;	x[2] = *y++;

//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//			R(i,j) = *y++;

//	P[0] = *y++;	P[1] = *y++;	P[2] = *y++;

//	L[0] = *y++;	L[1] = *y++;	L[2] = *y++;

//	v = P/m_Mass;

//	Iinv = R * Ibodyinv * R.transpose();
//	Vector3f tmp = Iinv * Vector3f(L);
//	omega = Vec3f(tmp[0], tmp[1], tmp[2]);
//}

//void SolidObject::array_to_bodies(float y[]) {
//	for (int i = 0; i < nBodies; i++) {
//		array_to_state(&Bodies[i], &y[i*state_size])
//	}
//}
void SolidObject::computeR(SolidObject* p) {
	float angle = 0;
	float dx = m_Position[0] - p->m_Position[0];
	float dy = m_Position[1] - p->m_Position[1];
	if (dy > 0) {
		Particle* p_high = this;
		Particle* p_low = p;
	}
	else {
		Particle* p_high = p;
		Particle* p_low = this;
	}

	if (dx > 0) {
		//this rechts van p
	}
	else {
		//this links van p
	}
	R(0, 0) = cosf(angle);
	R(0, 1) = -sinf(angle); 
	R(1, 0) = sinf(angle);
	R(1, 1) = cosf(angle);
	Vec3f x = m_Position;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			m_Position[i] += x[j] * R(i, j);
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			L[i] = Ibody(i, j) * omega[j];
		}
	}

	//v = P/m_Mass;
	Iinv = R * Ibodyinv * R.transpose();
	Vector3f tmp = Iinv * Vector3f(L);
	omega = Vec3f(tmp[0], tmp[1], tmp[2]);
}
void SolidObject::ddt_Calculations() {
	Matrix3f Rdot = star(omega) * R;
}
//{
//	*ydot++ = v[0];
//	*ydot++ = v[1];
//	*ydot++ = v[2];

//	Matrix3f Rdot = star(omega) * R;

//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//			*ydot++ = Rdot(i,j);

//	*ydot++ = force[0]; 
//	*ydot++ = force[1];
//	*ydot++ = force[2];

//	*ydot++ = torque[0]; 
//	*ydot++ = torque[1];
//	*ydot++ = torque[2];
//}

void SolidObject::draw()
{
	Vec3f x = m_Position;

	top_left = Vec3f(x[0], x[1] + yn * dist, 0); // top left
	top_right = Vec3f(x[0] + dist * xn, x[1] + yn * dist, 0); // top right 
	bottom_right = Vec3f(x[0] + dist * xn, x[1], 0); // bottom right
	bottom_left = Vec3f(x[0], x[1], 0); // bottom left

	glBegin(GL_QUADS);
	glColor3f(0.737255 , 0.560784 , 0.560784);
	glVertex2f(top_left[0],top_left[1]); // top left
	glVertex2f(top_right[0], top_right[1]); // top right 
	glVertex2f(bottom_right[0], bottom_right[1]); // bottom right
	glVertex2f(bottom_left[0], bottom_left[1]); // bottom left
	glEnd();
}

void SolidObject::init()
{
	R = Matrix3f::Identity();
	P = Vec3f(0, 0, 0);
	L = Vec3f(0, 0, 0);
	v = Vec3f(0, 0, 0);
	force = Vec3f(0, 0, 0);
	this->m_Force = force; //Particle
	torque = Vec3f(0, 0, 0);
	omega = Vec3f(0, 0, 0);
	m_Velocity = Vec3f(0, 0, 0);
	//m_Index = ps.size();
	m_OldPosition = m_Position;
	m_ConstructPos = m_Position;

	//init Ibody
	m_Mass = p_mass * xn * yn;
	Ibody = Matrix3f(3, 3);
	Ibody(0,0) = 1.0 / 12 * (m_Mass * ((yn * yn) + 1)); 
	Ibody(1,1) = 1.0 / 12 * (m_Mass * ((xn * xn) + 1));
	Ibody(2,2) = 1.0 / 12 * (m_Mass * ((xn * xn) + (yn * yn)));
	Ibodyinv = Ibody.inverse();
	Iinv = Ibodyinv;
	omega = eigen_to_vec( Iinv * Vector3f(L) );

	Vec3f x = m_Position;

	top_left = Vec3f(x[0], x[1] + yn * dist, 0); // top left
	top_right = Vec3f(x[0] + dist * xn, x[1] + yn * dist, 0); // top right 
	bottom_right = Vec3f(x[0] + dist * xn, x[1], 0); // bottom right
	bottom_left = Vec3f(x[0], x[1], 0); // bottom left

	/*create particles*/
	/*
	int cnt = ps.size(); //index of particle
	Vec3f o_pos = this->bot_left_pos;
	for (int j = 0; j < yn; j++) {
		for (int i = 0; i < xn; i++) {
			Vec3f r_pos = Vec3f(i * dist, j * dist, 0);
			Vec3f pos = o_pos + r_pos;
			Particle *p = new Particle(pos, p_mass, cnt);
			cnt++;
			ps.push_back(p);
			p_positions.push_back(r_pos);
			this->particles.push_back(p);
		}
	}
	*/

	/*rod connected implementation */
	/*
	for (int i = 0; i < xn; i++) {
		for (int j = 0; j < yn; j++) {
			auto p = particles[i + j*xn];
			int p_index = i + j*xn;

			//printf("\t current i,j (%d , %d)\n", i, j);
			int p2_index = i + 1 + j*xn;
			if (p2_index < particles.size() && i + 1!=xn) {
				//printf("\t adding left spring to (%d->%d)\n", p_index, p2_index);
				auto spring_left = new RodConstraint(p, particles[p2_index],
												   dist);
				cs.push_back(spring_left);
			}


			p2_index = i + (j + 1)*xn;
			if (p2_index < particles.size()) {
				//printf("\t adding up spring to (%d -> %d)\n", p_index, p2_index);
				auto spring_up = new RodConstraint(p, particles[p2_index],
												 dist);
				cs.push_back(spring_up);
			}
		}
	}
	*/
}

bool SolidObject::object_selected(Vec2f mouse)
{
	Vec3f x = m_Position;
	float dx = mouse[0] - x[0];
	float dy = mouse[1] - x[1];
	float d = sqrt(dx * dx + dy * dy);
	if (d < xn / 2.0 * dist && d < yn / 2.0 * dist) return true;
	return false;
}

void SolidObject::set_new_position(Vec3f mouse)
{
	float dv = 20.0;
	this->m_Velocity = dv * (mouse - m_Position);
}

void SolidObject::computeForce(Particle* p) {
	Vec3f tempVel = m_Velocity;
	if (p->getType() == "floor") {
		if (m_Velocity[1] <= 0) {
			m_Velocity[1] += 0.8*((m_Mass - p->m_Mass) / (m_Mass + p->m_Mass) - 1) * m_Velocity[1];
		}
		m_Force[1] += m_Mass * 9.81f;
	}
	else if (getType() == "floor") {
		if (p->m_Velocity[1] <= 0) {
			p->m_Velocity[1] += 0.8 * ((p->m_Mass - m_Mass) / (m_Mass + p->m_Mass) - 1) * p->m_Velocity[1];
		}
		p->m_Force[1] += p->m_Mass * 9.81f;
	}
	else {
		Vec3f forceDiff = m_Force + p->m_Force;
		m_Force += forceDiff;
		p->m_Force += forceDiff;
		m_Velocity = (m_Mass - p->m_Mass) / (m_Mass + p->m_Mass) * m_Velocity + (2 * p->m_Mass) / (m_Mass + p->m_Mass) * p->m_Velocity;
		p->m_Velocity = (p->m_Mass - m_Mass) / (m_Mass + p->m_Mass) * p->m_Velocity + (2 * m_Mass) / (m_Mass + p->m_Mass) * tempVel;
	}
		
	
}

void SolidObject::computeTorque()
{

}

bool SolidObject::is_collid(Particle* p)
{
	float x1 = m_Position[0];
	float x2 = m_Position[0] + xn*dist;

	float y1 = m_Position[1];
	float y2 = m_Position[1] + yn*dist;

	float x = p->m_Position[0];
	float y = p->m_Position[1];

	if (x > x1 && x < x2 && y > y1 && y < y2)
		return true;

	return false;

}

string SolidObject::getType()
{
	return type;
}

bool SolidObject::ObjectsCollide(SolidObject* so) {

	float x1l = so->m_Position[0];
	float x1r = so->m_Position[0] + so->xn * so->dist;

	float x2l = m_Position[0];
	float x2r = m_Position[0] + xn * dist;

	float y1l = so->m_Position[1];
	float y1u = so->m_Position[1] + so->yn * so->dist;

	float y2l = m_Position[1];
	float y2u = m_Position[1] + yn * dist;

	if (!(x1l >= x2r || x2l >= x1r)) {
		if (!(y1l >= y2u || y2l >= y1u)) {
			return true;
		};
	};
	return false;

}