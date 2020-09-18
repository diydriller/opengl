
#pragma once


#include "spring.h"
#include "Node.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <utility>
#include "Mesh.h"


using namespace std;



//class for face container
class int_mass
{
public:
	int first;
	int second;
	int third;

	int_mass(int x, int y, int z)
	{
		first = x;
		second = y;
		third = z;
	}
};











































class mass_cloth
{
public:

	std::vector<Node *> nodes;
	std::vector<mass_spring *> spring;
	std::vector<Node*> faces;
	//idx is face container
	std::vector<int_mass*> idx ;
	

	int			size_x, size_y, size_z;
	double		dx, dy, dz;
	double		structural_coef;
	double		shear_coef;
	double		bending_coef;
	int			iteration_n;
	int			drawMode;

	

	mass_cloth()
	{
		
	}
	~mass_cloth()
	{ 
		for (int i = 0; i < nodes.size(); i++){ delete nodes[i]; }
		for (int i = 0; i < spring.size(); i++){ delete spring[i]; }
		nodes.clear();
		spring.clear();
		faces.clear();
	}
	enum DrawModeEnum{
		DRAW_MASS_NODES,
		DRAW_SPRINGS,
		DRAW_FACES
	};
 
public:
	void init()
	{
		//node creation
		for (int i = 0; i < size_y; i++)
		{
			for (int j = 0; j < size_x; j++)
			{
				Node *xp = new Node(vec3(dx * j, dy * i + 10, dz * i));
				if (i == 0) 
					xp->isFixed = true;
				nodes.push_back(xp);
			}
		}

		//spring horizon creation
		for (int i = 0; i < size_y; i++)
		{
			for (int j = 0; j < size_x - 1; j++)
			{
				mass_spring *sp = new mass_spring(nodes[size_x * i + j], nodes[size_x * i + (j + 1)]);
				sp->spring_coef = structural_coef;
				spring.push_back(sp);
			}
		}

		//spring vertical creation
		for (int i = 0; i < size_y - 1; i++)
		{
			for (int j = 0; j < size_x; j++)
			{
				
				mass_spring *sp = new mass_spring(nodes[size_x * i + j], nodes[size_x * (i + 1) + j]);
				sp->spring_coef = structural_coef;
				spring.push_back(sp);
			}
		}

		//shear spring creation
		for (int i = 0; i < size_y - 1; i++)
		{
			for (int j = 0; j < size_x - 1; j++)
			{
				mass_spring *sp1 = new mass_spring(nodes[size_x * i + j], nodes[size_x * (i + 1) + (j + 1)]);
				mass_spring *sp2 = new mass_spring(nodes[size_x * (i + 1) + j], nodes[size_x * i + (j + 1)]);
				sp1->spring_coef = shear_coef;
				sp2->spring_coef = shear_coef;
				spring.push_back(sp1);
				spring.push_back(sp2);
			}
		}

		
		
		//face creation
		for (int i = 0; i < size_y - 1; i++)
		{
			for (int j = 0; j < size_x - 1; j++)
			{
				faces.push_back(nodes[size_x * (i + 1) + j]);
				faces.push_back(nodes[size_x * i + (j + 1)]);
				faces.push_back(nodes[size_x * (i + 1) + (j + 1)]);
				
				int_mass *f1 = new int_mass(size_x * (i+1) + j, size_x * i + (j+1) , size_x * (i + 1) + (j + 1));
				
				idx.push_back(f1);

				faces.push_back(nodes[size_x * (i + 1) + j]);
				faces.push_back(nodes[size_x * i + j]);		
				faces.push_back(nodes[size_x * i + (j + 1)]);
				
				

				int_mass *f2 = new int_mass(size_x * (i+1) + j, size_x * i + j, size_x * i + (j + 1));
				idx.push_back(f2);
				
				
			}
			
		}
	}
	

	void computeNormal()
	{

		

		for (int i = 0; i < idx.size(); i++)
		{
			vec3 f1= nodes[idx[i]->first]->position;
			vec3 f2 = nodes[idx[i]->second]->position;
			vec3 f3 = nodes[idx[i]->third]->position;

			vec3 fn = (f2 - f1).Cross(f3 - f1);
			fn = fn / fn.length();

			nodes[idx[i]->first]->normals.push_back(fn);
			nodes[idx[i]->second]->normals.push_back(fn);
			nodes[idx[i]->third]->normals.push_back(fn);

		}
		
		
		for (int i = 0; i < nodes.size(); i++)
		{
			vec3 vt_sum(0,0,0);

			for (int j = 0; j<nodes[i]->normals.size(); j++)
			{

				vt_sum += nodes[i]->normals[j];
			}
			
			nodes[i]->normal = vt_sum/vt_sum.length();
		}
		



		
		





	}
	
	void add_force(vec3 additional_force)
	{		 
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->add_force(additional_force);
		}
	}

	void compute_force(double dt, vec3 gravity)
	{
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->add_force(gravity * nodes[i]->mass);
		}
		/* Compute Force for all springs */
		for (int i = 0; i < spring.size(); i++)
		{
			spring[i]->internal_force(dt);
		}
	}

	void integrate(double dt)
	{
		/* integrate Nodes*/
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->integrate(dt);
		}
	}
	
	void collision_response(vec3 ground,Mesh *mesh1)
	{
		//Basic Implements 4. Collision Check with ground
		//Additional Implements 2. Collision Check with Sphere
		//Additional Implements 3. Collision Check with Mesh Object
		/*
			if(Collision Detection)
			{
				Collision Response
			}
		*/



		vec3 groundNormal(0, 1, 0);

		for (int i = 0; i < nodes.size(); i++)
		{



			if (nodes[i]->position.y <= ground.y && groundNormal.dot(nodes[i]->velocity))
			{
				vec3 v_n = groundNormal.dot(nodes[i]->velocity) * groundNormal;
				vec3 v_t = nodes[i]->velocity - v_n;
				
				nodes[i]->velocity = v_t - v_n;
			}

			/*

			for (int i = 0; i < mesh1->faceArray.size(); i++)
			{
				// face normal
				Vertex v0 = mesh1->vertexArray[mesh1->faceArray[i].vertex0];
				Vertex v1 = mesh1->vertexArray[mesh1->faceArray[i].vertex1];
				Vertex v2 = mesh1->vertexArray[mesh1->faceArray[i].vertex2];

				vec3 a1 = vec3(v0.normal.x, v0.normal.y, v0.normal.z);
				vec3 a2 = vec3(v0.position.x, v0.position.y, v0.position.z);
				vec3 b1 = vec3(v1.normal.x, v1.normal.y, v1.normal.z);
				vec3 b2 = vec3(v1.position.x, v1.position.y, v1.position.z);
				vec3 c1 = vec3(v2.normal.x, v2.normal.y, v2.normal.z);
				vec3 c2 = vec3(v2.position.x, v2.position.y, v2.position.z);
					
				vec3 v0temp = nodes[i]->position - a2;
				vec3 v1temp = nodes[i]->position -  b2;
				vec3 v2temp = nodes[i]->position - c2;


				if (v0temp.dot(a1) < 0.0f && a1.dot(nodes[i]->velocity) < 0.0f)
				{
					vec3 normalvelocity = a1.dot(nodes[i]->velocity)*a1;
					vec3 tempvelocity = nodes[i]->velocity - normalvelocity;
					nodes[i]->velocity = tempvelocity - normalvelocity;
				}

				if (v1temp.dot(b1) < 0.0f && b1.dot(nodes[i]->velocity) < 0.0f)
				{
					vec3 normalvelocity = b1.dot(nodes[i]->velocity)*b1;
					vec3 tempvelocity = nodes[i]->velocity - normalvelocity;
					nodes[i]->velocity = tempvelocity - normalvelocity;
				}

				if (v2temp.dot(c1) < 0.0f && c1.dot(nodes[i]->velocity) < 0.0f)
				{
					vec3 normalvelocity = c1.dot(nodes[i]->velocity)*c1;
					vec3 tempvelocity = nodes[i]->velocity - normalvelocity;
					nodes[i]->velocity = tempvelocity - normalvelocity;
				}



			}*/


		}
	}

	void draw();
};