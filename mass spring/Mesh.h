#pragma once

#include "Vertex.h"
#include "Face.h"
#include <fstream>
#include <string>


using namespace std;

class Mesh
{
public:
	vector<Vertex>	vertexArray;
	vector<Face>	faceArray;

	void			LoadMesh();
	void			ComputeFaceNormal();
	void			FindNeighborFaceArray();
	void			ComputeVertexNormal();
	void			draw();
};
