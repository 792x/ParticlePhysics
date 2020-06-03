// ParticleToy.cpp : Defines the entry point for the console application.

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Force.h"
#include "Constraint.h"
#include "GravityForce.h"
#include "AngularSpringForce.h"
#include "MouseForce.h"
#include "Cloth.h"

#include "solvers/Solver.h"
#include "solvers/AdaptiveTimeStepper.h"
#include "solvers/Euler.h"
#include "solvers/MidPoint.h"
#include "solvers/RungeKutta.h"
#include "solvers/BasicVerlet.h"
#include "solvers/LeapFrog.h"
#include "solvers/ConstraintSolver.h"

#include "Hair.h"

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
static float d, c_dt = 0.01f;
static int dsim;
static int dump_frames;
static int frame_number;
static Vec3f mouse_position;
static int particle_selected = -1;


// static Particle *pList;
static std::vector<Particle *> pVector;
static std::vector<Force *> fVector;
static std::vector<Constraint *> cVector;
static std::vector<Object*> oVector;
static std::vector<MouseForce *> mVector;

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
static Euler implEuler = Euler(Euler::impl);
static MidPoint MidPointSolver;
static RungeKutta RungeKuttaSolver;
static BasicVerlet BasicVerletSolver;
static LeapFrog LeapFrogSolver;
static Solver* solvers[7] = {&explEuler, &semiEuler, &implEuler, &MidPointSolver, &RungeKuttaSolver, &BasicVerletSolver, &LeapFrogSolver};
static int solverIndex = 0;
bool Solver::simulation_reset = true;

// global variables used for adaptive time stepping
static AdaptiveTimeStepper adaptiveTimeStepper(c_dt);
static bool adaptive_time_stepping = false;

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

	for (Object *o : oVector) {
		delete o;
	}
	oVector.clear();

}

static void clear_data() {
	int ii, size = pVector.size();

	for (ii = 0; ii < size; ii++) {
		pVector[ii]->reset();
	}

	adaptiveTimeStepper.reset(fVector);
	Solver::simulation_reset = true;
}

static void init_system() {
	const double dist = 0.2;
	const Vec3f center(0.0, 0.0, 0.0);
	const Vec3f offset(0.0, dist, 0.0);

	//oVector.push_back(new Hair(pVector, fVector, cVector));
	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.
	pVector.push_back(new Particle(center - offset, 1.0f, 0));
	pVector.push_back(new Particle(center - offset - offset, 1.0f, 1));
	pVector.push_back(new Particle(center - offset - offset - offset, 1.0f, 2));

	Cloth c = Cloth(5, 7, Vec3f(0.2f,0.2f,0.2f), pVector, fVector, cVector, 1.0f, 0.08f, 8000, 100);

	fVector.push_back(new GravityForce(pVector));
	fVector.push_back(new SpringForce(pVector[0], pVector[1], dist, 500,  0.5));
	fVector.push_back(new SpringForce(pVector[1], pVector[2], dist, 500,  0.5));


	//fVector.push_back(new AngularSpringForce({ pVector[0],pVector[1],pVector[2] }, PI, 120.0, 100.0));

	//cVector.push_back(new RodConstraint(pVector[0], pVector[1], dist));
	//cVector.push_back(new CircularWireConstraint(pVector[0], center, dist));

	//fVector.push_back(new SpringForce(pVector[3], pVector[4], dist, 150.0f, 1.50f));
	//fVector.push_back(new SpringForce(pVector[4], pVector[5], dist, 150.0f, 1.50f));
	//cVector.push_back(new RodConstraint(pVector[3], pVector[4], dist));
	//cVector.push_back(new CircularWireConstraint(pVector[3], center, dist));



	//GravityForce gravity_force = GravityForce(pVector);
	/*SpringForce spring1 = SpringForce(pVector[0], pVector[1], dist, 1, 1);
	SpringForce spring2 = SpringForce(pVector[1], pVector[2], dist, 1, 1);
	AngularSpringForce asf = AngularSpringForce(pVector, dist, 120.0, 100.0);*/


	//gravity_force.apply();

	for (int i = 0; i < pVector.size(); i++) {
		mVector.push_back(new MouseForce(pVector[i], pVector[i]->m_Velocity, 100, 0.5));
	}
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
	for (MouseForce *m : mVector) {
		m->draw();
	}
}

static void draw_constraints() {

	for (Constraint *c : cVector) {
		c->draw();
	}
}

static void draw_objects() {

	for (Object *o : oVector) {
		o->draw();
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
	float x, y = 0;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0]
		&& !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx/(float)win_x)*N);
	j = (int)(((win_y - my)/(float)win_y)*N);

	if (i < 1 || i > N || j < 1 || j > N) return;

	if (mouse_down[0]) {
		x = i - 32.0f;
		x = x / 32.0f;
		y = j - 32.0f;
		y = y / 32.0f;

		for (i = 0; i < pVector.size(); i++) {
			mouse_position[0] = x;
			mouse_position[1] = y;
			float delta_x = pVector[i]->m_Position[0] - mouse_position[0];
			float delta_y = pVector[i]->m_Position[1] - mouse_position[1];
			float dist = delta_x*delta_x + delta_y*delta_y;
			if (dist < 0.003) {
				particle_selected = i;
			}

			if (particle_selected == i) {
				std::cout << i << std::endl;
				mVector[i]->set_mouse(mouse_position);
				mVector[i]->apply();
			} else {
				mVector[i]->set_mouse(pVector[i]->m_Position);
			}
		}
	} else {
		particle_selected = -1;
		int i, size = pVector.size();
		for (i = 0; i < size; i++) {
			mVector[i]->set_mouse(pVector[i]->m_Position);
		}
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
			printf("\t Using solver 3. Implicit Euler\n");
			break;

		case '4': solverIndex = 3;
			printf("\t Using solver 4. Explicit MidPoint\n");
			break;

		case '5': solverIndex = 4;
			printf("\t Using solver 5. Explicit Runge Kutta\n");
			break;

		case '6': solverIndex = 5;
			Solver::simulation_reset = true;
			printf("\t Using solver 6. Basic Verlet\n");
			break;

		case '7': solverIndex = 6;
			Solver::simulation_reset = true;
			printf("\t Using solver 7. Leapfrog\n");
			break;

		case '+': adaptive_time_stepping = true;
			adaptiveTimeStepper.reset(fVector);
			printf("\t Using adaptive time stepping\n");
			break;

		case '-': adaptive_time_stepping = false;
			printf("\t Using constant time stepping\n");
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
		get_from_UI();

		// Using adaptive time stepping with changing dt.
		if (adaptive_time_stepping) {
			adaptiveTimeStepper.simulation_step(pVector, fVector, cVector, solvers[solverIndex]);

		// Using constant time stepping with c_dt as defined before compilation.
		}else {
			solvers[solverIndex]->simulation_step(pVector, fVector, cVector, c_dt);
		}
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
	draw_objects();

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
		//c_dt = 10.f; //Moved to top for initialisation of the adaptive time stepper
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, c_dt, d);
	} else {
		N = atoi(argv[1]);
		c_dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Switch between solvers by pressing the keys '1' and '2'\n");
	printf("\t Available solvers:\n");
	printf("\t 1. Explicit Euler\n");
	printf("\t 2. Semi Implicit Euler\n");
	printf("\t 3. Implicit Euler\n");
	printf("\t 4. Explicit MidPoint\n");
	printf("\t 5. Explicit Runge Kutta\n");
	printf("\t 6. Basic Verlet\n");
	printf("\t 7. Leapfrog\n");
	printf("\t Switch between adaptive time stepping and constant time stepping by using the + and the - keys\n");
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
