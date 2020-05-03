#include "SpringForce.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::apply()
{
    Vec2f x = m_p1->m_Position - m_p2->m_Position;
    Vec2f v = m_p1->m_Velocity - m_p2->m_Position;
    Vec2f f = x / norm(x);
    f *= (m_ks * (norm(x) - m_dist) + m_kd * ((x * v) / norm(x)));
    m_p1->m_Force -= f;
    m_p2->m_Force += f;
}
