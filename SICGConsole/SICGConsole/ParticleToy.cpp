// ParticleToy.cpp : Defines the entry point for the console application.

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Force.h"
#include "Constraint.h"
#include "GravityForce.h"
#include "AngularSpringForce.h"
#include "solvers/Solver.h"
#include "solvers/Euler.h"
#include "solvers/MidPoint.h"
#include "Cloth.h"
#include "solvers/ConstraintSolver.h"
#include "solvers/RungeKutta.h"
#include "solvers/BasicVerlet.h"

//#include "imageio.h"

#include <vector>
#include <cstdlib>
#include <cstdio>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static std::vector<Particle *> pVector;
static std::vector<Force *> fVector;
static std::vector<Constraint *> cVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

// global variables used for solvers
static Euler explEuler = Euler(Euler::expl);
static Euler semiEuler = Euler(Euler::semi);
static MidPoint MidPointSolver;
static RungeKutta RungeKuttaSolver;
static BasicVerlet BasicVerletSolver;
static Solver* solvers[5] = {&explEuler, &semiEuler, &MidPointSolver, &RungeKuttaSolver, &BasicVerletSolver};
static int solverIndex = 2;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data() {
	for (Force *f : fVector) {
		delete f;
	}
	fVector.clear();

	for (Particle *p : pVector) {
		delete p;
	}
	pVector.clear();

	for (Constraint *c : cVector) {
		delete c;
	}
	cVector.clear();

}

static void clear_data() {
	int ii, size = pVector.size();

	for (ii = 0; ii < size; ii++) {
		pVector[ii]->reset();
	}

}

static void init_system() {
	const double dist = 0.2;
	const Vec3f center(0.0, 0.0, 0.0);
	const Vec3f offset(dist, 0.0, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.
	pVector.push_back(new Particle(center + offset, 1.0f, 0));
	pVector.push_back(new Particle(center + offset + offset, 1.0f, 1));
	pVector.push_back(new Particle(center + offset + offset + offset, 1.0f, 2));

	Cloth c = Cloth(5, 7, Vec3f( 0.2f,0.2f,0.2f ), pVector, fVector, cVector,1.0f,0.08f);

	fVector.push_back(new GravityForce(pVector));
	fVector.push_back(new SpringForce(pVector[0], pVector[1], dist, 1, 1));
	fVector.push_back(new SpringForce(pVector[1], pVector[2], dist, 1, 1));


	cVector.push_back(new RodConstraint(pVector[0], pVector[1], dist));
	cVector.push_back(new CircularWireConstraint(pVector[0], center, dist));

	//GravityForce gravity_force = GravityForce(pVector);
	/*SpringForce spring1 = SpringForce(pVector[0], pVector[1], dist, 1, 1);
	SpringForce spring2 = SpringForce(pVector[1], pVector[2], dist, 1, 1);
	AngularSpringForce asf = AngularSpringForce(pVector, dist, 120.0, 100.0);*/


	//gravity_force.apply();



}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display() {
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display() {
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number%FRAME_INTERVAL)==0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char *buffer = (unsigned char *)malloc(w*h*4*sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			//sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			//saveImageRGBA(filename, buffer, w, h);

			free(buffer);
		}
	}
	frame_number++;

	glutSwapBuffers();
}

static void apply_forces() {
	// Reset all the forces.
	for (Force* f : fVector) {
		f->reset();
	}

	// Compute and apply all the new forces.
	for (Force* f : fVector) {
		f->apply();
	}

}
//
//static void apply_constraints() {
//	ConstraintSolver::solve(pVector, cVector, 100.0f, 10.0f);
//};

static void draw_particles() {
	int size = pVector.size();

	for (int ii = 0; ii < size; ii++) {
		pVector[ii]->draw();
	}
}

static void draw_forces() {
	// change this to iteration over full set
	for (Force *f : fVector) {
		f->draw();
	}
}

static void draw_constraints() {

	for (Constraint *c : cVector) {
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI() {
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0]
		&& !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx/(float)win_x)*N);
	j = (int)(((win_y - my)/(float)win_y)*N);

	if (i < 1 || i > N || j < 1 || j > N) return;

	if (mouse_down[0]) {

	}

	if (mouse_down[2]) {
	}

	hi = (int)((hmx/(float)win_x)*N);
	hj = (int)(((win_y - hmy)/(float)win_y)*N);

	if (mouse_release[0]) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI() {
	int ii, size = pVector.size();
	for (ii = 0; ii < size; ii++) {
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y) {
	switch (key) {
		case '1': solverIndex = 0;
			printf("\t Using solver 1. Explicit Euler\n");
			break;

		case '2': solverIndex = 1;
			printf("\t Using solver 2. Semi-implicit Euler\n");
			break;

		case '3': solverIndex = 2;
			printf("\t Using solver 3. Explicit MidPoint\n");
			break;

		case '4': solverIndex = 3;
			printf("\t Using solver 4. Explicit Runge Kutta\n");
			break;

		case '5': solverIndex = 4;
			printf("\t Using solver 5. Basic Verlet\n");
			break;

		case 'c':
		case 'C': clear_data();
			break;

		case 'd':
		case 'D': dump_frames = !dump_frames;
			break;

		case 'q':
		case 'Q': free_data();
			exit(0);
			break;

		case ' ': dsim = !dsim;
			break;
		default:
			break;
	}
}

static void mouse_func(int button, int state, int x, int y) {
	omx = mx = x;
	omx = my = y;

	if (!mouse_down[0]) {
		hmx = x;
		hmy = y;
	}
	if (mouse_down[button]) mouse_release[button] = state==GLUT_UP;
	if (mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state==GLUT_DOWN;
}

static void motion_func(int x, int y) {
	mx = x;
	my = y;
}

static void reshape_func(int width, int height) {
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func() {
	if (dsim) {
		apply_forces();
//		apply_constraints();
		solvers[solverIndex]->simulation_step(pVector, fVector, cVector, dt);
	} else {
		get_from_UI();
		remap_GUI();
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func() {
	pre_display();

	draw_forces();
	draw_constraints();
	draw_particles();

	post_display();
}

/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window() {
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Particletoys!");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv) {
	glutInit(&argc, argv);

	if (argc==1) {
		N = 64;
		dt = 0.01f;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, dt, d);
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Switch between solvers by pressing the keys '1' and '2'\n");
	printf("\t Available solvers:\n");
	printf("\t 1. Explicit Euler\n");
	printf("\t 2. Semi Implicit Euler\n");
	printf("\t 3. Explicit MidPoint\n");
	printf("\t 4. Explicit Runge Kutta\n");
	printf("\t 5. Basic Verlet\n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	init_system();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}

