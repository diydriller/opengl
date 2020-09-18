#pragma once
#include "Particle.h"

using namespace std;
#define SPH_PI 3.1415926
#define GRIDSIZE 40

class SPH
{
public:
	vector<Particle *> particles;
	int index;				//particle index
	int MaxParticle;
	int iteration_n;
private:
	double rest_density;	// rest density
	double mu;				// viscosity constant
	double h;				// kernel radius
	double k;				// gas constant
	
public:
	SPH();
	SPH(int numparticle);
	~SPH();
	
	void resetParticle();
	void init();
	void damBreaking();
	void pouring();
	void update(float dt, vec3 gravity);
	void draw();
	void computeDensity();
	void computeForce();
	void integrate(double dt, vec3 gravity);
	void collision_response(vec3 ground);

};