#pragma once

#include <vector>
#include "Particle.h"

using namespace std;
class Force{
public:
	virtual void apply()=0;
	virtual void draw()=0;
//private;
//	vector<Particle*> particles;

};