#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "stdafx.h"
#include "GL/glut.h"
#endif

#include <utility>
#include "GravityForce.h"
#include "solvers/ConstraintSolver.h"

const Vec3f GravityForce::standard_gravity = Vec3f(0.0, -9.81f, 0.0);

GravityForce::GravityForce(std::vector<Particle *> particles) {
	this->target(std::move(particles));
}

void GravityForce::target(std::vector<Particle *> particles) {
	this->particles = particles;
}

void GravityForce::apply() {
	for (Particle *p : particles) {
		//std::cout << "mass: " << p->m_Mass << std::endl;
		p->m_Force += p->m_Mass*GravityForce::standard_gravity;
	}
}

void GravityForce::draw() {
}
