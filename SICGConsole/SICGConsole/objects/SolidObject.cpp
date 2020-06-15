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

SolidObject::SolidObject(int x, int y, Vec3f bottom_left_pos, std::string type, float p_mass, float dist) :
	xn(x), yn(y), bot_left_pos(bottom_left_pos), type(type), p_mass(p_mass), dist(dist)
{
	/*particle implementation of solid object*/
	m_Position = bottom_left_pos;
	/*particle implementation of solid object*/
	init();
}

void SolidObject::computeR(Vec3f rotationCenter, float angle)
{
	top_left[0] = (top_left[0] - rotationCenter[0]) * cosf(angle) + (top_left[1] - rotationCenter[1]) * sinf(angle) + rotationCenter[0];
	top_left[1] = (top_left[0] - rotationCenter[0]) * (-sinf(angle)) + (top_left[1] - rotationCenter[1]) * cosf(angle) + rotationCenter[1];
	top_right[0] = (top_right[0] - rotationCenter[0]) * cosf(angle) + (top_right[1] - rotationCenter[1]) * sinf(angle) + rotationCenter[0];
	top_right[1] = (top_right[0] - rotationCenter[0]) * (-sinf(angle)) + (top_right[1] - rotationCenter[1]) * cosf(angle) + rotationCenter[1];
	bottom_right[0] = (bottom_right[0] - rotationCenter[0]) * cosf(angle) + (bottom_right[1] - rotationCenter[1]) * sinf(angle) + rotationCenter[0];
	bottom_right[1] = (bottom_right[0] - rotationCenter[0]) * (-sinf(angle)) + (bottom_right[1] - rotationCenter[1]) * cosf(angle) + rotationCenter[1];
	bottom_left[0] = (bottom_left[0] - rotationCenter[0]) * cosf(angle) + (bottom_left[1] - rotationCenter[1]) * sinf(angle) + rotationCenter[0];
	bottom_left[1] = (bottom_left[0] - rotationCenter[0]) * (-sinf(angle)) + (bottom_left[1] - rotationCenter[1]) * cosf(angle) + rotationCenter[1];

}
void SolidObject::ddt_Calculations() {
	Matrix3f Rdot = star(omega) * R;
}

void SolidObject::draw()
{
	Vec3f x = m_Position;
	glBegin(GL_QUADS);

	glColor3f(0.737255, 0.560784, 0.560784);
	glVertex2f(x[0] + top_left[0], x[1] + top_left[1]); // top left
	glVertex2f(x[0] + top_right[0], x[1] + top_right[1]); // top right 
	glVertex2f(x[0] + bottom_right[0], x[1] + bottom_right[1]); // bottom right
	glVertex2f(x[0] + bottom_left[0], x[1] + bottom_left[1]); // bottom left

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
	m_OldPosition = m_Position;
	m_ConstructPos = m_Position;

	//init Ibody
	m_Mass = p_mass * xn * yn;
	Ibody = Matrix3f(3, 3);
	Ibody(0, 0) = 1.0 / 12 * (m_Mass * ((yn * yn) + 1));
	Ibody(1, 1) = 1.0 / 12 * (m_Mass * ((xn * xn) + 1));
	Ibody(2, 2) = 1.0 / 12 * (m_Mass * ((xn * xn) + (yn * yn)));
	Ibodyinv = Ibody.inverse();
	Iinv = Ibodyinv;
	omega = eigen_to_vec(Iinv * Vector3f(L));

	Vec3f x = m_Position;

	top_left = Vec3f(0, yn * dist, 0); // top left
	top_right = Vec3f(dist * xn, yn * dist, 0); // top right 
	bottom_right = Vec3f(dist * xn, 0, 0); // bottom right
	bottom_left = Vec3f(0, 0, 0); // bottom left
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
			m_Velocity[1] += 0.6 * ((m_Mass - p->m_Mass) / (m_Mass + p->m_Mass) - 1) * m_Velocity[1];
		}
		m_Force[1] += m_Mass * 9.81f;
	}
	else if (getType() == "floor") {
		if (p->m_Velocity[1] <= 0) {
			p->m_Velocity[1] += 0.6 * ((p->m_Mass - m_Mass) / (m_Mass + p->m_Mass) - 1) * p->m_Velocity[1];
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

void SolidObject::computeForceObject(SolidObject* p) {
	float distL = p->m_Position[0] + p->top_left[0] - m_Position[0] - bottom_left[0];
	float distR = m_Position[0] + bottom_right[0] - p->m_Position[0] - p->top_left[0];

	if (distL > distR) {
		computeR(p->m_Position + p->top_left, -0.0005);
	}
}

void SolidObject::computeTorque()
{

}

// Check if the solid object collided.
bool SolidObject::is_collid(Particle* p)
{
	float x1 = m_Position[0];
	float x2 = m_Position[0] + xn * dist;

	float y1 = m_Position[1];
	float y2 = m_Position[1] + yn * dist;

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

bool onSegment(Vec3f p, Vec3f q, Vec3f r)
{
	if (q[0] <= max(p[0], r[0]) && q[0] >= min(p[0], r[0]) &&
		q[1] <= max(p[1], r[1]) && q[1] >= min(p[1], r[1]))
		return true;

	return false;
}

// Check the orientation of the polygon.
int orientation(Vec3f p, Vec3f q, Vec3f r)
{
	int val = (q[1] - p[1]) * (r[0] - q[0]) -
		(q[0] - p[0]) * (r[1] - q[1]);

	if (val == 0) return 0;  // colinear 

	return (val > 0) ? 1 : 2; // clock or counterclock wise 
}

bool doIntersect(Vec3f p1, Vec3f q1, Vec3f p2, Vec3f q2)
{
	// Find the four orientations needed for general and 
	// special cases 
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case 
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases 
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1 
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and q2 are colinear and q2 lies on segment p1q1 
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2 
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2 
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases 
}
bool SolidObject::ObjectsCollide(SolidObject* so) {
	if (so->bottom_left[1] == so->bottom_right[1] && bottom_left[1] == bottom_right[1]) {
		float x1l = so->m_Position[0];
		float x1r = so->m_Position[0] + so->xn * so->dist;

		float x2l = m_Position[0];
		float x2r = m_Position[0] + xn * dist;

		if (!(x1l >= x2r || x2l >= x1r)) {

			float y1l = so->m_Position[1];
			float y1u = so->m_Position[1] + so->yn * so->dist;

			float y2l = m_Position[1];
			float y2u = m_Position[1] + yn * dist;

			if (!(y1l >= y2u || y2l >= y1u)) {
				return true;
			};
		};
		return false;
	}

	std::vector<std::vector<Vec3f>> sides = {};
	sides.push_back({ m_Position + bottom_right,  m_Position + bottom_left });
	sides.push_back({ m_Position + bottom_left,  m_Position + top_left });
	sides.push_back({ m_Position + top_left,  m_Position + top_right });
	sides.push_back({ m_Position + top_right,  m_Position + bottom_right });

	std::vector<std::vector<Vec3f>> sides2 = {};
	sides2.push_back({ so->m_Position + so->bottom_right, so->m_Position + so->bottom_left });
	sides2.push_back({ so->m_Position + so->bottom_left, so->m_Position + so->top_left });
	sides2.push_back({ so->m_Position + so->top_left, so->m_Position + so->top_right });
	sides2.push_back({ so->m_Position + so->top_right, so->m_Position + so->bottom_right });


	for (std::vector<Vec3f> s1 : sides) {
		for (std::vector<Vec3f> s2 : sides2) {
			if (doIntersect(s1[0], s1[1], s2[0], s2[1])) {
				return true;
			}
		}
	}

	return false;
}