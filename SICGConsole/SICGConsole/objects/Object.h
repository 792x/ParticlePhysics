#pragma once
#include "Particle.h"
#include "../forces/Force.h"
#include "../constraints/Constraint.h"

#include<vector>

using namespace std;
class Object
{
public:
    virtual void draw()=0 ;
   vector<Particle*> particles;
    void set_new_position(Vec3f mouse);
};

