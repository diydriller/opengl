#pragma once

#include "VECTOR1.h"

class Face
{

public:
	int vertex0;
	int vertex1;
	int vertex2;

	VECTOR3D v0;
	VECTOR3D v1;
	VECTOR3D v2;

	VECTOR3D normal;
};