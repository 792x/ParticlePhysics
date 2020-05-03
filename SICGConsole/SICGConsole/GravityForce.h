#pragma once

#include <vector>
#include "Particle.h"
#include "Force.h"

using namespace std;

class GravityForce : public Force{
public:
    GravityForce(vector<Particle*> ps);

    void draw() override;
    void apply() override;
private:
    vector<Particle*> particles;
    Vec2f const g(0,-9.8);
};

