#include "SolidObject.h"
#include <Eigen/Dense>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
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

SolidObject::SolidObject(int x, int y, Vec3f bottom_left_pos, vector<Particle*>& ps, vector<Force*>& fs, 
	vector<Constraint*>& cs, float p_mass, float dist):
	xn(x),yn(y),bot_left_pos(bottom_left_pos),p_mass(p_mass),dist(dist),x(bottom_left_pos)
{
	init(ps, fs, cs);
}

void SolidObject::state_to_array(float* y)
{
	*y++ = x[0];	*y++ = x[1];	*y++ = x[2];

	for (int i = 0; i < 3; i++) /* copy rotation matrix */
		for (int j = 0; j < 3; j++)
			*y++ = R(i,j);

	*y++ = P[0];	*y++ = P[1];	*y++ = P[2];

	*y++ = L[0];	*y++ = L[1];	*y++ = L[2];
}

void SolidObject::array_to_state(float* y)
{
	x[0] = *y++;	x[1] = *y++;	x[2] = *y++;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R(i,j) = *y++;

	P[0] = *y++;	P[1] = *y++;	P[2] = *y++;

	L[0] = *y++;	L[1] = *y++;	L[2] = *y++;

	v = P;

	Iinv = R * Ibodyinv * R.transpose();
	Vector3f tmp = Iinv * Vector3f(L);
	omega = Vec3f(tmp[0], tmp[1], tmp[2]);
}

void SolidObject::ddt_State_to_Array(float* ydot)
{
	*ydot++ = v[0];
	*ydot++ = v[1];
	*ydot++ = v[2];

	Matrix3f Rdot = star(omega) * R;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			*ydot++ = Rdot(i,j);

	*ydot++ = force[0]; 
	*ydot++ = force[1];
	*ydot++ = force[2];

	*ydot++ = torque[0]; 
	*ydot++ = torque[1];
	*ydot++ = torque[2];
}

void SolidObject::draw()
{
	glBegin(GL_QUADS);
	
	glColor3f(0.0, 1.0, 0.0);
	Vec3f tl = particles[particles.size()-xn]->m_Position;
	Vec3f tr = particles[particles.size()-1]->m_Position;
	Vec3f br = particles[xn-1]->m_Position;
	Vec3f bl = particles[0]->m_Position;
	glVertex2f(tl[0],tl[1]); // top left
	glVertex2f(tr[0],tr[1]); // top right 
	glVertex2f(br[0],br[1]); // bottom right
	glVertex2f(bl[0],bl[1]); // bottom left
	glEnd();
}

void SolidObject::init(vector<Particle*>& ps, vector<Force*>& fs, vector<Constraint*>& cs)
{
	P = Vec3f(0, 0, 0);
	L = Vec3f(0, 0, 0);
	v = Vec3f(0, 0, 0);
	force = Vec3f(0, 0, 0);
	torque = Vec3f(0, 0, 0);
	omega = Vec3f(0, 0, 0);

	//init Ibody
	this->mass = p_mass * xn * yn;
	Ibody = Matrix3f(3, 3);
	Ibody(0,0) = 1.0 / 12 * (mass * ((yn * yn) + 1)); 
	Ibody(1,1) = 1.0 / 12 * (mass * ((xn * xn) + 1));
	Ibody(2,2) = 1.0 / 12 * (mass * ((xn * xn) + (yn * yn)));
	Ibodyinv = Ibody.inverse();
	Iinv = Ibodyinv;
	omega = eigen_to_vec( Iinv * Vector3f(L) );


	//Matrix3f tmp;
	//for (int i = 0; i < 3; i++) {
	//	for (int j = 0; j < 3; j++) {
	//		tmp(i, j) = Ibody[i][j];
	//	}
	//}

	//Matrix3f inv = tmp.inverse();
	//for (int i = 0; i < 3; i++) {
	//	for (int j = 0; j < 3; j++) {
	//		Ibodyinv[i][j] = inv(i,j);
	//	}
	//}



	/*create particles*/
	int cnt = ps.size(); //index of particle
	Vec3f o_pos = this->bot_left_pos;
	for (int j = 0; j < yn; j++) {
		for (int i = 0; i < xn; i++) {
			Vec3f r_pos = Vec3f(i * dist, j * dist, 0);
			Vec3f pos = o_pos + r_pos;
			Particle *p = new Particle(pos, p_mass, cnt,false);
			cnt++;
			//ps.push_back(p);
			p_positions.push_back(r_pos);
			this->particles.push_back(p);
		}
	}
}

bool SolidObject::object_selected(Vec2f mouse)
{
	//cout << "checking object selected x (" << x[0] << "," << x[1] << ")" << endl;
	//cout << "checking object selected mouse (" << mouse[0] << "," << mouse[1] << ")" << endl;
	float dx = mouse[0] - x[0];
	float dy = mouse[1] - x[1];
	float d = sqrt(dx * dx + dy * dy);
	if (d < xn / 2.0 * dist && d < yn / 2.0 * dist) return true;
	return false;
}

void SolidObject::set_new_position(Vec3f mouse)
{
	//cout << "setting new position" << endl;
	//cout << "original x (" << x[0] << "," << x[1] << ")" << endl;
	x[0] = mouse[0];
	x[1] = mouse[1];
	//cout << "new x (" << x[0] << "," << x[1] << ")" << endl;
	for (int i = 0; i < particles.size(); i++) {
		particles[i]->m_Position = x + p_positions[i];
	}
}

void SolidObject::computeForce()
{
	force = Vec3f(0, 0, 0);
	for (auto p : particles) {
		force += p->m_Force;
	}
}

void SolidObject::computeTorque()
{
	torque = Vec3f(0, 0, 0);
	for (auto p : particles) {
		torque += cross(p->m_Position - x, p->m_Force);
	}

}