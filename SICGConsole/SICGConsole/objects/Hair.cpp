#include "Hair.h"
#include "../forces/SpringForce.h"
#include "../forces/AngularSpringForce.h"
#include "../constraints/CircularWireConstraint.h"
#include "../constraints/RodConstraint.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "../GL/glut.h"
#endif

static void draw_circle(const Vec3f& vect, float radius) {
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < 360; i = i + 18) {
		float degInRad = i * PI / 180;
		glVertex2f(vect[0] + cos(degInRad) * radius, vect[1] + sin(degInRad) * radius);
	}
	glEnd();
}

// Hair constructor.
Hair::Hair(vector<Particle*>& pVector, vector<Force*>& fVector, vector<Constraint*>& cVector,
	float p_ks, float p_kd,
	Vec3f center, float mass, float radius) :
	center(center), ks(p_ks), kd(p_kd), mass(mass), radius(radius) {
	const Vec3f mini_offset(0.0, 0.01, 0.0);

	cout << "angle " << angle << " cos:" << cos(angle) << endl;
	for (float angle : angles) {
		float dx = cos(angle) * radius;
		float dy = sin(angle) * radius;


		Vec3f pos_l = center + Vec3f{ -dx, dy, 0 };
		Particle* p_l = new Particle(pos_l, mass, pVector.size(), false);
		pVector.push_back(p_l);
		particles.push_back(p_l);
		cVector.push_back(new CircularWireConstraint(p_l, pos_l - mini_offset, 0.02f));


		float dist = 0.2;
		for (int i = 1; i < particles_per_hair; i++) {
			pos_l += offset_l;
			Particle* pl = new Particle(pos_l, mass, pVector.size(), false);
			pVector.push_back(pl);
			fVector.push_back(new SpringForce(pl, particles[particles.size() - 1], hair_particle_dist, ks, kd));

			particles.push_back(pl);
			if ((i + 1) % 3 == 0 && i - 2 >= 0) {
				int j = particles.size();
				fVector.push_back(new AngularSpringForce({ particles[j - 3],
					particles[j - 2],particles[j - 1] }, angle, ang_ks, ang_kd));
			}

		}

		Vec3f pos_r = center + Vec3f{ dx, dy, 0 };
		Particle* p_r = new Particle(pos_r, mass, pVector.size(), false);
		pVector.push_back(p_r);
		particles.push_back(p_r);
		cVector.push_back(new CircularWireConstraint(p_r, pos_r - mini_offset, 0.02f));

		for (int i = 1; i < particles_per_hair; i++) {
			pos_r += offset_r;
			Particle* pr = new Particle(pos_r, mass, pVector.size(), false);
			pVector.push_back(pr);
			fVector.push_back(new SpringForce(pr, particles[particles.size() - 1], hair_particle_dist, ks, kd));

			particles.push_back(pr);
			if ((i + 1) % 3 == 0 && i - 2 >= 0) {
				int j = particles.size();
				fVector.push_back(new AngularSpringForce({ particles[j - 3],
					particles[j - 2],particles[j - 1] }, angle, ang_ks, ang_kd));
			}
		}
	}
}

void Hair::draw()
{
	draw_circle(center, radius);
}

void Hair::addForces(vector<Force*>& fVector)
{
}

void Hair::addConstraints(vector<Constraint*>& cVector)
{
}
