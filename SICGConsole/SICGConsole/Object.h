#pragma once
#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

#include<vector>
using namespace std;
class Object
{
public:
    virtual void draw()=0 ;
   vector<Particle*> particles;
};

