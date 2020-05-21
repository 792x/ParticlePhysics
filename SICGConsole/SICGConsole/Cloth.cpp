#include "Cloth.h"
#include "SpringForce.h"
#include "CircularWireConstraint.h"

using namespace std;

Cloth::Cloth(int x, int y, Vec3f pos, vector<Particle*>& ps, 
	vector<Force*>& fs, vector<Constraint*>& cs, float mass , float dist,
	float ks, float kd):
	xn(x), yn(y), c_ks(ks), c_kd(kd)
{
	this->bot_left_pos = pos;
	this->dist = dist;
	init(ps, fs, cs, mass);
}

void Cloth::init(vector<Particle *> &ps,
				 vector<Force *> &fs,
				 vector<Constraint *> &cs,
				 float mass) {
	int cnt = ps.size();
	Vec3f o_pos = this->bot_left_pos;
	for (int j = 0; j < yn; j++) {
		for (int i = 0; i < xn; i++) {
			Vec3f pos = o_pos + Vec3f(i*dist, j*dist, 0);
			//Particle *p = new Particle(pos, mass, j + i*xn);
			Particle *p = new Particle(pos, mass, cnt);
			cnt++;
			ps.push_back(p);
			this->particles.push_back(p);
		}
	}
	addConstraints(ps, cs);
	addForces(ps, fs);
}

void Cloth::addForces(vector<Particle*> ps, vector<Force*>& fs)
{
	for (int i = 0; i < xn; i++) {
		for (int j = 0; j < yn; j++) {

			// . --- .      spring between four particles
			// |\  / | 
			// | \/  |
			// | /\  |
			// |/  \ |
			// . --- .
			auto p = particles[i + j*xn];
			int p_index = i + j*xn;

			//printf("\t current i,j (%d , %d)\n", i, j);
			int p2_index = i + j*xn + 1;
			if (p2_index < particles.size() && i + 1!=xn) {
				//printf("\t adding left spring to (%d->%d)\n", p_index, p2_index);
				auto spring_left = new SpringForce(p, particles[p2_index],
					dist, c_ks, c_kd);
				fs.push_back(spring_left);
			}

			p2_index = i + (j + 1) * xn;
			if (p2_index < particles.size()) {
				//printf("\t adding up spring to (%d -> %d)\n", p_index, p2_index);
				auto spring_up = new SpringForce(p, particles[p2_index],
					dist, c_ks, c_kd);
				fs.push_back(spring_up);
			}

			p2_index = i + 1 + (j + 1)*xn;
			if (p2_index < particles.size() && i + 1!=xn) {
				//printf("\t adding left up spring to (%d -> %d)\n", p_index, p2_index);
				auto spring_left_up = new SpringForce(p, particles[p2_index],
					dist * sqrt(2), c_ks, c_kd);
				fs.push_back(spring_left_up);
			}

			p2_index = i + 1 + (j - 1)*xn;
			if (p2_index < particles.size() && p2_index > 0 && i + 1!=xn) {
				//printf("\t adding left down spring to (%d -> %d)\n", p_index, p2_index);
				auto spring_down_left = new SpringForce(p, particles[p2_index],
					dist * sqrt(2), c_ks, c_kd);
				fs.push_back(spring_down_left);
			}
		}
	}
}

void Cloth::addConstraints(vector<Particle*> ps, vector<Constraint*>& cs)
{
	double r = 0.002f;
	Vec3f o_pos = this->bot_left_pos;

	//the top left and right point to fix the cloth
	auto c_left = new CircularWireConstraint(particles[yn - 1],
		o_pos + Vec3f((xn - 1)* dist, (yn - 1)* dist, 0), r);
	auto c_right = new CircularWireConstraint(particles[particles.size() - 1],
		o_pos + Vec3f(dist* (xn - 1), dist* (yn - 1), 0), r);
	cs.push_back(c_left);
	cs.push_back(c_right);

	/*
	for (int i = 0; i < xn; i++) {
		for (int j = 0; j < yn; j++) {
			Vec3f pos = o_pos + Vec3f(i * dist, j * dist, 0);
			//Particle* p = new Particle(pos, mass, j+i*xn);
			//ps.push_back(p);
			//this->particles.push_back(p);
			//auto p = particles[j * xn + i];
			//auto c = new 
		}
	}
	*/
}

void Cloth::draw()
{
	for (auto p : particles) {
		p->draw();
	}
}

Cloth::~Cloth()
{
}

