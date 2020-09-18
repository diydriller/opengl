#include "SPH.h"
#include <time.h>

SPH::SPH()
{

}

SPH::SPH(int numparticle)
{
	MaxParticle = numparticle;
	index = 0;	
	rest_density = 4.2;
	mu = 0.8;	
	h = 1.0;
	k = 50.0;
}

SPH::~SPH()
{
	while (!particles.empty())
	{
		particles.pop_back();
	}
	particles.clear();
}

void SPH::resetParticle()
{
	index = 0;
	while (!particles.empty())
	{
		particles.pop_back();
	}
}

void SPH::init()
{
	resetParticle();
	damBreaking();
}

void SPH::damBreaking()
{
	for (double z = 0; z < 10.0; z += 1)
	{
		for (double y = 10; y < 20.0; y += 1) {
			for (double x = 0; x < 10.0; x += 1) {
				if (particles.size() < MaxParticle)
				{
					Particle *p = new Particle(x, y, z, index++);
					particles.push_back(p);
				}
			}
		}
	}
	cout << "SPH" << particles.size() << " Paricles" << endl;
}

void SPH::pouring()
{
	if (particles.size() >= MaxParticle)
		return;
	for (double z = 0; z < 10.0; z += 2) {

		for (double y = 0; y < 10.0; y += 2) {
			for (double x = 0; x < 10.0; x += 2) {
				if (particles.size() < MaxParticle)
				{
					Particle *p = new Particle(x, y,0, index++);
					p->velocity.x = 15.0f;
					particles.push_back(p);
				}
			}
		}
	}
	cout << "SPH" << particles.size() << " Paricles" << endl;
}

void SPH::collision_response(vec3 ground)
{
	for (int i = 0; i < particles.size(); i++)
	{
		if (particles[i]->getPosY() <= ground.y)
		{
			particles[i]->position.y = ground.y;	
		}
	}
}


void SPH::update(float dt, vec3 gravity)
{

	

	computeDensity();
	computeForce();
	integrate(dt, gravity);
	collision_response(gravity);
}

void SPH::draw()
{
	for (int i = 0; i < particles.size(); i++)
	{
		Particle *p = particles[i];
		p->draw();
	}
}


void SPH::computeDensity()
{
	for (int x = 0; x < GRIDSIZE; x++)
	{
		for (int y = 0; y < GRIDSIZE; y++)
		{
			for (int z = 0; z < GRIDSIZE; z++)
			{
				vector<Particle*> ris;


				for (int i = 0; i < ris.size(); i++)
				{
					Particle *pi = ris[i];
					pi->density = 1.0;	//compute with poly6Kernel

					/*Implements - Compute Density 작성, 아래 density 초기화는 지울 것*/

					/*
					for (int j = 0; j < rjs.size(); j++)
					{
						Particle *pj = rjs[j];
						vec2 rij = pi->position - pj->position;
						double q = rij.dist() / h;
						if (0.0 <= q && q < 1.0)
						{
							pi->density = pi->density + pj->mass * poly6Kernel(rij, h);
						}
					}
					*/

				}
			}
		}
	}
}

void SPH::computeForce() // Compute Pressure and Viscosity
{
	for (int x = 0; x < GRIDSIZE; x++)
	{
		for (int y = 0; y < GRIDSIZE; y++)
		{
			for (int z = 0; z < GRIDSIZE; z++)
			{
				vector<Particle*> ris;


				for (int i = 0; i < ris.size(); i++)
				{
					Particle *pi = ris[i];
					pi->fpressure = vec3(0.0, 0.0, 0.0);//compute with spikygradientKernel
					pi->fviscosity = vec3(0.0, 0.0, 0.0);//compute with viscositylaplacianKernel

					/*Implements - Compute Pressure and Viscosity Forces 작성*/

					/*
					for (int j = 0; j < rjs.size(); j++)
					{
						Particle *pj = rjs[j];
						vec2 rij = pi->position - pj->position;
						double q = rij.dist() / h;
						if (0.0 <= q && q < 1.0)
						{
							pi->fpressure = pi->fpressure + pj->mass
								*(k*((pi->density - rest_density) + (pj->density - rest_density)) / (2.0*pj->density))
								* spikygradientKernel(rij, q);
							pi->fviscosity = pi->fviscosity + pj->mass
								*((pj->velocity - pi->velocity) / pj->density)
								*viscositylaplacianKernel(rij, q);
						}

					}
					pi->fpressure = -1.0 * pi->fpressure;
					pi->fviscosity = mu * pi->fviscosity;
					*/

				}
			}
		}
	}
}

void SPH::integrate(double dt, vec3 gravity)
{
	for (int i = 0; i < particles.size(); i++)
	{
		Particle *p = particles[i];
		p->integrate(dt, gravity);
	}
}

