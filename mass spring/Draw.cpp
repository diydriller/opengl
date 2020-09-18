#include "Node.h"

#include <GL/glut.h>
#include "cloth.h"
#include "Mesh.h"
#include "Particle.h"

#pragma warning(disable:4996)
#define STB_IMAGE_IMPLEMENTATION
#include "GL/stb_image.h"


GLuint mTexture;

bool LoadMeshFromFile(const char* texFile)
{
	glGenTextures(1, &mTexture);
	FILE *fp = fopen(texFile, "rb");

	int width, height, channel;
	unsigned char *image = stbi_load_from_file(fp, &width, &height, &channel, 4);
	fclose(fp);

	glBindTexture(GL_TEXTURE_2D, mTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, GL_MODULATE);
	
	return true;
}


void Node::draw()
{
	glDisable(GL_LIGHTING);
	glColor3f(0.97, 0.95, 0.15);
	glPointSize(2.0);

	glBegin(GL_POINTS);	
	glVertex3f(getPosX(), getPosY(), getPosZ());
	glEnd();
	glEnable(GL_LIGHTING);
}

void mass_spring::draw()
{
	glDisable(GL_LIGHTING);
	glColor3f(1.0, 1.0, 1.0);
	glLineWidth(2.0);

 	glBegin(GL_LINES);
	glVertex3f(p1->position.x, p1->position.y, p1->position.z);
	glVertex3f(p2->position.x, p2->position.y, p2->position.z);
  	glEnd();	 
	glEnable(GL_LIGHTING);

}

void Mesh::draw()
{
	
	
	glPushMatrix();
	glTranslatef(-25.0, -25.0, 0.0);

	for (int i = 0; i < faceArray.size(); i++)
	{
		// face normal
		glNormal3f(faceArray[i].normal.x, faceArray[i].normal.y,  faceArray[i].normal.z);
		Vertex v0 =  vertexArray[ faceArray[i].vertex0];
		Vertex v1 =  vertexArray[ faceArray[i].vertex1];
		Vertex v2 = vertexArray[ faceArray[i].vertex2];
		glBegin(GL_POLYGON);

		// vertex normal
		//glNormal3f(v0.normal.x, v0.normal.y, v0.normal.z);
		glVertex3f(v0.position.x, v0.position.y, v0.position.z);
		//glNormal3f(v1.normal.x, v1.normal.y, v1.normal.z);
		glVertex3f(v1.position.x, v1.position.y, v1.position.z);
		//glNormal3f(v2.normal.x, v2.normal.y, v2.normal.z);
		glVertex3f(v2.position.x, v2.position.y, v2.position.z);
		glEnd();

	}
	glPopMatrix();
}

void mass_cloth::draw()
{	
	switch (drawMode)
	{
	case DRAW_MASS_NODES:
		glDisable(GL_LIGHTING);
		for (int i = 0; i < nodes.size(); i++)
			nodes[i]->draw();













		glEnable(GL_LIGHTING);
		break;
	case DRAW_SPRINGS:
		
		for (int i = 0; i < spring.size(); i++)
			spring[i]->draw();
		glEnable(GL_LIGHTING);
		break;
	case DRAW_FACES:
		//Basic Implements 3-3. Draw Call for Cloth
		//Additional Implements 4-3. Texture Coordinate Mapping
		
		LoadMeshFromFile("image1.jpg");
		
		
		
		for (int i = 0; i < idx.size(); i++)
		{
			vec3 f11 = nodes[idx[i]->first]->position;
			vec3 f12 = nodes[idx[i]->first]->normal;

			vec3 f21 = nodes[idx[i]->second]->position;
			vec3 f22 = nodes[idx[i]->second]->normal;

			vec3 f31 = nodes[idx[i]->third]->position;
			vec3 f32 = nodes[idx[i]->third]->normal;

			glEnable(GL_TEXTURE_2D);
			glBegin(GL_POLYGON);
			  
			    glTexCoord2f((idx[i]->first % size_x) / (float)size_x, 1.0f - idx[i]->first / (float)size_y / (float)size_y);
			    glNormal3f(f12.x, f12.y, f12.z);
				glVertex3f(f11.x, f11.y, f11.z);
				
				glTexCoord2f((idx[i]->second % size_x) / (float)size_x, 1.0f - idx[i]->second / (float)size_y / (float)size_y);
				glNormal3f(f22.x, f22.y, f22.z);
				glVertex3f(f21.x, f21.y, f21.z);
				
				glTexCoord2f((idx[i]->third % size_x) / (float)size_x, 1.0f - idx[i]->third / (float)size_y / (float)size_y);
				glNormal3f(f32.x, f32.y, f32.z);
				glVertex3f(f31.x, f31.y, f31.z);
				
			glEnd();
			glDisable(GL_TEXTURE_2D);
		}
		
		
		break;
	default:
		break;
	}
	glPopMatrix();
}

void Particle::draw()
{

	glPushMatrix();
	glTranslatef(-25.0, -25.0, 0.0);

	glColor3f(1.0f, 1.0f, 1.0f);
	//glutSolidSphere(0.1f, 100, 100);
	
	glPointSize(2.0f);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	glVertex3f(getPosX(), getPosY(),getPosZ());
	//cout << getPosX() << "," << getPosY() << "," << getPosZ() << endl;
	glEnd();
	
	glPopMatrix();
}