#include "Cloth.h"
#include "../forces/SpringForce.h"
#include "../constraints/CircularWireConstraint.h"

using namespace std;

Cloth::Cloth(int x, int y, Vec3f pos, vector<Particle*>& ps,
	vector<Force*>& fs, vector<Constraint*>& cs, float mass, float dist,
	float ks, float kd) :
	xn(x), yn(y), c_ks(ks), c_kd(kd) {
	this->bot_left_pos = pos;
	this->dist = dist;
	init(ps, fs, cs, mass);
}

// Initialize the cloth.
void Cloth::init(vector<Particle*>& ps,
	vector<Force*>& fs,
	vector<Constraint*>& cs,
	float mass) {
	int cnt = ps.size();
	Vec3f o_pos = this->bot_left_pos;
	for (int j = 0; j < yn; j++) {
		for (int i = 0; i < xn; i++) {
			Vec3f pos = o_pos + Vec3f(i * dist, j * dist, 0);
			//Particle *p = new Particle(pos, mass, j + i*xn);
			Particle* p = new Particle(pos, mass, cnt);
			cnt++;
			ps.push_back(p);
			this->particles.push_back(p);
		}
	}
	addForces(ps, fs);
	addConstraints(ps, cs);
}

// Add spring forces to the cloth.
void Cloth::addForces(vector<Particle*> ps, vector<Force*>& fs) {
	for (int i = 0; i < xn; i++) {
		for (int j = 0; j < yn; j++) {

			// . --- .      spring between four particles
			// |\  / | 
			// | \/  |
			// | /\  |
			// |/  \ |
			// . --- .
			auto p = particles[i + j * xn];
			int p_index = i + j * xn;

			int p2_index = i + 1 + j * xn;
			if (p2_index < particles.size() && i + 1 != xn) {
				auto spring_left = new SpringForce(p, particles[p2_index],
					dist, c_ks, c_kd);
				fs.push_back(spring_left);
			}

			p2_index = i + 2 + j * xn;
			if (p2_index < particles.size() && i + 2 < xn) {
				auto spring_left_2 = new SpringForce(p, particles[p2_index],
					2 * dist, c_ks, c_kd);
				fs.push_back(spring_left_2);
			}

			p2_index = i + (j + 1) * xn;
			if (p2_index < particles.size()) {
				auto spring_up = new SpringForce(p, particles[p2_index],
					dist, c_ks, c_kd);
				fs.push_back(spring_up);
			}

			p2_index = i + (j + 2) * xn;
			if (p2_index < particles.size()) {
				auto spring_up_2 = new SpringForce(p, particles[p2_index],
					2 * dist, c_ks, c_kd);
				fs.push_back(spring_up_2);
			}

			p2_index = i + 1 + (j + 1) * xn;
			if (p2_index < particles.size() && i + 1 != xn) {
				auto spring_left_up = new SpringForce(p, particles[p2_index],
					dist * sqrt(2), c_ks, c_kd);
				fs.push_back(spring_left_up);
			}

			p2_index = i + 1 + (j - 1) * xn;
			if (p2_index < particles.size() && p2_index > 0 && i + 1 != xn) {
				auto spring_down_left = new SpringForce(p, particles[p2_index],
					dist * sqrt(2), c_ks, c_kd);
				fs.push_back(spring_down_left);
			}
		}
	}
}

// Add constraints to the top left and right particles of the cloth.
void Cloth::addConstraints(vector<Particle*> ps, vector<Constraint*>& cs) {
	double r = 0.03f;
	Vec3f o_pos = this->bot_left_pos;
	const Vec3f mini_offset(0.0, 0.02, 0.0);

	auto c_left = new CircularWireConstraint(ps[xn * yn + (ps.size() - this->particles.size()) - xn],
		o_pos + Vec3f(0, dist * (yn - 1), 0) + mini_offset, r);
	auto c_right = new CircularWireConstraint(ps[xn * yn + (ps.size() - this->particles.size()) - 1],
		o_pos + Vec3f(dist * (xn - 1), dist * (yn - 1), 0) + mini_offset, r);
	cs.push_back(c_left);
	cs.push_back(c_right);
}

void Cloth::draw() {
	//this should be done in global drawing
}

Cloth::~Cloth() {
}