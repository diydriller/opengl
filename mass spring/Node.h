

#pragma once
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "vector.h"
class Node
{
public:
	double	mass;
	vec3	force;
	vec3	position;
	vec3	velocity;
	vec3	acceleration;
	vec3    normal;
	std::vector<vec3> normals;
	bool	isFixed;

public:
	Node(void)
	{
		isFixed = false;
		
	}
	Node(vec3 init_pos)
	{
		isFixed = false;
		position = init_pos;
		mass = 1.0;
	}
 
	~Node(void)
	{
	}

	double	getPosX(void) { return position.getX(); }
	double	getPosY(void) { return position.getY(); }
	double	getPosZ(void){ return position.getZ(); }

	void add_force(vec3 additional_force)
	{
		force += additional_force;
	}

	void integrate(double dt)
	{
		if (!isFixed)
		{
			//Basic Implements 2-2. Integration
			acceleration = force / mass;

			vec3 temp_vel = acceleration * dt;
			velocity = velocity + temp_vel;

			vec3 temp_po = velocity * dt;
			position = position + temp_po;
		}
		/*initialize Force*/
		force.x = force.y = force.z= 0.0;
	}

	void draw();
};
