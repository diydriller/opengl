

#pragma once

#include <iostream>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "cloth.h"
#include  "Mesh.h"
#include "SPH.h"

using namespace std;


class Simulator
{
public:
	Simulator(void);
	~Simulator(void);

public:
	void					Initialize(void);
 	void					Update();
	void					Render();
	void					Lighting(void);
	void					DrawGround(void);




public:
	mass_cloth			*cloth;
	vec3				ground;
	float timsStep;
	Mesh				*mesh;
	SPH					*mySPH;
	
};

