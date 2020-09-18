#include <iostream>
#include <stdlib.h>
#include <set>
#include <vector>
#include "GL/freeglut.h"
#include "Box2D/Box2D.h"
#include "SDL/SDL_keyboard.h"

using namespace std;


int scr_width = 640;
int scr_height = 640;

b2World* world;
b2Body* box;
b2Body* map[12];
b2PulleyJoint* m_joint;
b2Body* body1 = NULL;
b2Body* body2 = NULL;

b2Body* body3;
b2Body* body4;

b2Body* body5;
b2Body* body6;

b2RevoluteJoint* m_joint1;
b2RevoluteJoint* m_joint2;

b2Body* water;

b2Vec2 gravity;



//b2CircleShape boxshape;
b2PolygonShape boxshape;
b2PolygonShape mapshape[12];
b2PolygonShape boxshape1,boxshape2,boxshape3,boxshape4,boxshape5,boxshape6;
b2PolygonShape watershape;

int32 velocityIterations = 8;
int32 positionIterations = 3;

float32 g_hz = 60.0f;
float32 timeStep = 1.0f / g_hz;



#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f


float xx = 0;

typedef pair<b2Fixture *, b2Fixture *>fixturePair;

class b2ContactListener_ : public b2ContactListener
{
public:
	set<fixturePair> m_fixturePairs;
	b2ContactListener_() {};

	void BeginContact(b2Contact * contact)
	{
		b2Fixture * fixtureA = contact->GetFixtureA();
		b2Fixture *fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
		{
			m_fixturePairs.insert(make_pair(fixtureB, fixtureA));

		}
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
		{
			m_fixturePairs.insert(make_pair(fixtureA, fixtureB));

		}
	}

	void EndContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
		{
			m_fixturePairs.erase(make_pair(fixtureB, fixtureA));
		}
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
		{
			m_fixturePairs.erase(make_pair(fixtureA, fixtureB));
		}
	}

	
};


void mapset() {




	
	//a static body
	b2BodyDef mapbd[12];
	b2FixtureDef myFixtureDef[12];
	b2Body* staticBody[12];

	for (int i = 0; i < 12; i++)
	{
		mapbd[i].type = b2_staticBody;
		mapbd[i].position.Set(0, 0);

		staticBody[i] = world->CreateBody(&mapbd[i]);


		//fixture definition
	
		myFixtureDef[i].shape = &mapshape[i];
	}

	//add four walls to the static body
	mapshape[0].SetAsBox(100, 1, b2Vec2(0, 0), 0);//ground
	staticBody[0]->CreateFixture(&myFixtureDef[0]);
	mapshape[1].SetAsBox(100, 1, b2Vec2(0, 60), 0);//ceiling
	staticBody[1]->CreateFixture(&myFixtureDef[1]);
	mapshape[2].SetAsBox(1, 30, b2Vec2(-100, 30), 0);//left wall
	staticBody[2]->CreateFixture(&myFixtureDef[2]);
	mapshape[3].SetAsBox(1, 30, b2Vec2(100, 30), 0);//right wall
	staticBody[3]->CreateFixture(&myFixtureDef[3]);
	mapshape[4].SetAsBox(1, 20, b2Vec2(-80, 20),0);//ground
	staticBody[4]->CreateFixture(&myFixtureDef[4]);
	mapshape[5].SetAsBox(1, 20, b2Vec2(-60, 40), 0);//ground
	staticBody[5]->CreateFixture(&myFixtureDef[5]);
	mapshape[6].SetAsBox(1, 20, b2Vec2(-40, 20), 0);//ground
	staticBody[6]->CreateFixture(&myFixtureDef[6]);
	mapshape[7].SetAsBox(1, 20, b2Vec2(-20, 46), 45*DEGTORAD);//ground
	staticBody[7]->CreateFixture(&myFixtureDef[7]);
	mapshape[8].SetAsBox(1, 10, b2Vec2(0, 10), 0);//ground
	staticBody[8]->CreateFixture(&myFixtureDef[8]);
	mapshape[9].SetAsBox(1, 10, b2Vec2(40, 10), 0);//ground
	staticBody[9]->CreateFixture(&myFixtureDef[9]);
	mapshape[10].SetAsBox(1, 15, b2Vec2(50, 50), 0);//ground
	staticBody[10]->CreateFixture(&myFixtureDef[10]);
	mapshape[11].SetAsBox(1, 15, b2Vec2(50, 10), 0);//ground
	staticBody[11]->CreateFixture(&myFixtureDef[11]);
	for (int i = 0; i < 12; i++)
	{
		map[i] = staticBody[i];
	}
}

void pulleyset()
{


	b2Vec2 groundanchor1, groundanchor2;
	b2Vec2 anchor1, anchor2;

	groundanchor1.Set(55.0f, 50.0f);
	groundanchor2.Set(70.0f, 50.0f);

	float short_length = 10.0f;
	float32 box_w = 4.0f;
	float32 box_h = 4.0f;

	boxshape1.SetAsBox(box_w, box_h);
	boxshape2.SetAsBox(box_w, box_h);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(groundanchor1.x, groundanchor1.y - short_length - box_h);
	body1 = world->CreateBody(&bd);
	body1->CreateFixture(&boxshape1, 0.4f);

	bd.position.Set(groundanchor2.x, groundanchor2.y - short_length - box_h);
	body2 = world->CreateBody(&bd);
	body2->CreateFixture(&boxshape2, 0.4f);

	float ratio = 1.0f;
	b2PulleyJointDef pulleyDef;
	anchor1.Set(groundanchor1.x, groundanchor1.y - short_length);
	anchor2.Set(groundanchor2.x, groundanchor2.y - short_length);
	pulleyDef.Initialize(body1, body2, groundanchor1, groundanchor2, anchor1, anchor2, ratio);
	m_joint = (b2PulleyJoint*)world->CreateJoint(&pulleyDef);


}

void waterset()
{
	b2BodyDef boxbd_water;
	boxbd_water.type = b2_staticBody;
	boxbd_water.position.Set(20.0f, 8.0f);

	b2Body*body_water = world->CreateBody(&boxbd_water);

	watershape.SetAsBox(19.0f, 8.0f);

	b2FixtureDef boxfd_water;
	boxfd_water.shape = &watershape;
	boxfd_water.density = 4.0f;
	boxfd_water.restitution = 0.0f;
	boxfd_water.friction = 5.0f;
	boxfd_water.isSensor = true;
	body_water->CreateFixture(&boxfd_water);
	water = body_water;


}

b2ContactListener_ contactListener;


void Setup() {
	
	gravity.Set(0.0f, -10.0f);

	world = new b2World(gravity);

	

	mapset();
	pulleyset();
	
	b2BodyDef boxbd;
	boxbd.type = b2_dynamicBody;
	boxbd.position.Set(-95.0f, 5.0f);
	b2Body* body = world->CreateBody(&boxbd);
	boxshape.SetAsBox(1.5f,1.5f) ;
	//boxshape.m_radius = 0.6f;
	b2FixtureDef boxfd;
	boxfd.shape = &boxshape;
	boxfd.density = 1.0f;
	boxfd.restitution = 0.4f;
	boxfd.friction = 0.4f;
	body->CreateFixture(&boxfd);
	box = body;
	box->SetLinearVelocity(b2Vec2(10.0f, 0.0f));
	box->SetAngularVelocity(-90 * DEGTORAD);


	waterset();

	world->SetContactListener(&contactListener);

	












	b2BodyDef bodyDef1;
	b2BodyDef bodyDef2;
	bodyDef1.type = b2_dynamicBody;
	bodyDef2.type = b2_staticBody;
	b2FixtureDef fixtureDef1;
	b2FixtureDef fixtureDef2;
	fixtureDef1.density = 0.1f;
	fixtureDef1.restitution = 0.1f;
	fixtureDef1.friction = 0.1f;
	fixtureDef2.density = 1;

	//two shapes
	//b2PolygonShape boxShape;
	boxshape3.SetAsBox(5, 0.5);
	//b2CircleShape circleShape;
	//circleShape.m_radius = 2;
	boxshape4.SetAsBox(0.1, 0.1);

	//make box a little to the left
	bodyDef1.position.Set(-95, 10);
	fixtureDef1.shape = &boxshape3;
	body3 = world->CreateBody(&bodyDef1);
	body3->CreateFixture(&fixtureDef1);
	body3->SetGravityScale(0);

	//and circle a little to the right
	bodyDef2.position.Set(-90, 20);
	fixtureDef2.shape = &boxshape4;
	body4 = world->CreateBody(&bodyDef2);
	body4->CreateFixture(&fixtureDef2);
	body4->SetGravityScale(0);

	
	b2RevoluteJointDef revoluteJointDef;

	revoluteJointDef.enableMotor = true;
	revoluteJointDef.maxMotorTorque = 10000;
	revoluteJointDef.motorSpeed = 500;//90 degrees per second

	revoluteJointDef.bodyA = body3;
	revoluteJointDef.bodyB = body4;
	revoluteJointDef.collideConnected = false;
	revoluteJointDef.localAnchorA.Set(0, 0);//the top right corner of the box
	revoluteJointDef.localAnchorB.Set(0, 0);//center of the circle
	m_joint1 = (b2RevoluteJoint*)world->CreateJoint(&revoluteJointDef);
	
	











	b2BodyDef bodyDef3;
	b2BodyDef bodyDef4;
	bodyDef3.type = b2_dynamicBody;
	bodyDef4.type = b2_staticBody;
	b2FixtureDef fixtureDef3;
	b2FixtureDef fixtureDef4;
	fixtureDef3.density = 0.1f;
	fixtureDef3.restitution = 0.1f;
	fixtureDef3.friction = 0.1f;
	fixtureDef4.density = 1;

	//two shapes
	//b2PolygonShape boxShape;
	boxshape5.SetAsBox(5, 0.5);
	//b2CircleShape circleShape;
	//circleShape.m_radius = 2;
	boxshape6.SetAsBox(0.1, 0.1);

	//make box a little to the left
	bodyDef3.position.Set(-75, 10);
	fixtureDef3.shape = &boxshape5;
	body5 = world->CreateBody(&bodyDef3);
	body5->CreateFixture(&fixtureDef3);
	body5->SetGravityScale(0);

	//and circle a little to the right
	bodyDef4.position.Set(-70, 20);
	fixtureDef4.shape = &boxshape6;
	body6 = world->CreateBody(&bodyDef4);
	body6->CreateFixture(&fixtureDef4);
	body6->SetGravityScale(0);


	b2RevoluteJointDef revoluteJointDef1;

	
	revoluteJointDef1.bodyA = body5;
	revoluteJointDef1.bodyB = body6;
	revoluteJointDef1.collideConnected = false;
	revoluteJointDef1.localAnchorA.Set(0, 0);//the top right corner of the box
	revoluteJointDef1.localAnchorB.Set(0, 0);//center of the circle
	m_joint2 = (b2RevoluteJoint*)world->CreateJoint(&revoluteJointDef1);
}



bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p)
{
	return (cp2.x - cp1.x)*(p.y - cp1.y) > (cp2.y - cp1.y)*(p.x - cp1.x);

}


b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e)
{
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);
	float n1 = cp1.x*cp2.y - cp1.y*cp2.x;
	float n2 = s.x*e.y - s.y *e.x;
	float n3 = 1.0 / (dc.x*dp.y - dc.y*dp.x);
	return b2Vec2((n1*dp.x - n2 * dc.x)*n3, (n1*dp.y - n2 * dc.y)*n3);


}

bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices)
{
	if (fA->GetShape()->GetType() != b2Shape::e_polygon ||
		fB->GetShape()->GetType() != b2Shape::e_polygon)
		return false;
	b2PolygonShape* polyA = (b2PolygonShape*)fA->GetShape();
	b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();

	for (int i = 0; i < polyA->GetVertexCount(); i++)
		outputVertices.push_back(fA->GetBody()->GetWorldPoint(polyA->GetVertex(i)));

	vector<b2Vec2> clipPolygon;
	for (int i = 0; i < polyB->GetVertexCount(); i++)
		clipPolygon.push_back(fB->GetBody()->GetWorldPoint(polyB->GetVertex(i)));

	b2Vec2 cp1 = clipPolygon[clipPolygon.size() - 1];
	for (int j = 0; j < clipPolygon.size(); j++) {
		b2Vec2 cp2 = clipPolygon[j];
		if (outputVertices.empty())
			return false;
		vector<b2Vec2> inputList = outputVertices;
		outputVertices.clear();
		b2Vec2 s = inputList[inputList.size() - 1];
		for (int i = 0; i < inputList.size(); i++) {
			b2Vec2 e = inputList[i];
			if (inside(cp1, cp2, e)) {
				if (!inside(cp1, cp2, s)) {
					outputVertices.push_back(intersection(cp1, cp2, s, e));
				}
				outputVertices.push_back(e);
			}
			else if (inside(cp1, cp2, s)) {
				outputVertices.push_back(intersection(cp1, cp2, s, e));

			}
			s = e;
		}
		cp1 = cp2;
	}
	return !outputVertices.empty();
}



b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area)
{
	int count = (int)vs.size();
	b2Assert(count >= 3);

	b2Vec2 c;
	c.Set(0.0f, 0.0f);
	area = 0.0f;
	b2Vec2 pRef(0.0f, 0.0f);
	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i)
	{
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);
		float32 triangleArea = 0.5f*D;
		area += triangleArea;

		c+=triangleArea*inv3*(p1 + p2 + p3);
	}
	if (area > b2_epsilon)
		c *= 1.0f / area;
	else
		area = 0;
	return c;
}

void applybuoyancy(b2Fixture* box, b2Fixture* water, float area, b2Vec2 gravity, b2Vec2 centroid)
{
	float displacedMass = water->GetDensity()*area;
	b2Vec2 buoyancyForce = displacedMass * -1 * gravity;
	box->GetBody()->ApplyForce(buoyancyForce, centroid, true);
}

void applydrag(b2Fixture* box, b2Fixture* water, float area, b2Vec2 centroid)
{
	b2Vec2 velDir = box->GetBody()->GetLinearVelocityFromWorldPoint(centroid) -
		water->GetBody()->GetLinearVelocityFromWorldPoint(centroid);

	float vel = velDir.Normalize();

	float dragMag = water->GetDensity()*vel*vel / 2;
	b2Vec2 dragForce = dragMag * -velDir;
	box->GetBody()->ApplyForce(dragForce, centroid, true);

	float angularDrag = area * -water->GetBody()->GetAngularVelocity();
	box->GetBody()->ApplyTorque(angularDrag, true);

}

void Update(int value) {

	
	world->Step(timeStep, velocityIterations, positionIterations);
	//b2Vec2 position = box->GetPosition();
	//std::cout << "Box position (" << position.x << ", " << position.y << ")" << std::endl;
	//body3->ApplyForce(b2Vec2(0, 160.0f), body3->GetWorldCenter(), true);
	//body3->ApplyTorque(100,1);
	//body4->ApplyForce(b2Vec2(0, 160.0f), body4->GetWorldCenter(), true);

	//body4->ApplyForce(b2Vec2(0,160.0f), body4->GetWorldCenter(), true);
	


	if (contactListener.m_fixturePairs.size() > 0)
	{
		std::cout << "o" << endl;
		set<fixturePair>::iterator it = contactListener.m_fixturePairs.begin();
		set<fixturePair>::iterator end = contactListener.m_fixturePairs.end();

		while (it != end)
		{
			b2Fixture* fixture_box = it->first;
			b2Fixture* fixture_water = it->second;

			float density = fixture_water->GetDensity();

			vector<b2Vec2> intersectionPoints;

			if (findIntersectionOfFixtures(fixture_box, fixture_water, intersectionPoints))
			{

				float area = 0;
				b2Vec2 centroid = ComputeCentroid(intersectionPoints, area);

				applybuoyancy(fixture_box, fixture_water, area, gravity, centroid);
				applydrag(fixture_box, fixture_water, area, centroid);

			}
			++it;
		}
	}

	
	glutPostRedisplay();
	glutTimerFunc(20, Update, 0);
}

void Render()
{
	
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	

	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	b2Vec2 boxpos = box->GetPosition();
	gluLookAt(boxpos.x, boxpos.y/2, 0.0f, boxpos.x, boxpos.y/2, -1.0f, 0.0f, 1.0f, 0.0f);




	

	





	
	




	for (int i = 0; i < 12; i++)
	{

	    glPushMatrix();


		b2Vec2 backpos = map[i]->GetPosition();
		float32 backang = map[i]->GetAngle();

		glTranslatef(backpos.x, backpos.y, 0.0f);
		glRotatef(backang, 0.0f, 0.0f, 1.0f);
		glColor3f(0.0f, 1.0f, 0.0f);
		
		glBegin(GL_QUADS);
		for (int j = 0; j < 4; j++) {
			glVertex2f(mapshape[i].m_vertices[j].x, mapshape[i].m_vertices[j].y);

		}
		glEnd();
		



		glPopMatrix();

	}






	glPushMatrix();

	b2Vec2 pos = box->GetPosition();
	float32 ang = box->GetAngle();

	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(ang, 0.0f, 0.0f, 1.0f);
	glColor3f(0.9f, 0.2f, 0.4f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape.m_vertices[i].x, boxshape.m_vertices[i].y);

	}
	glEnd();
/*
	double rad = 0.6;

	glBegin(GL_POLYGON);
	for (int i = 0; i < 360; i++)
	{
		double angle = i * DEGTORAD;
		double x = rad * cos(angle);
		double y = rad * sin(angle);
		glVertex2f(x, y);
	}
	glEnd();
	*/
	glPopMatrix();



	
	glPushMatrix();

	b2Vec2 pos1 = body1->GetPosition();
	float a1 = body1->GetAngle();

	glTranslatef(pos1.x, pos1.y, 0.0f);
	glRotatef(a1, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape1.m_vertices[i].x, boxshape1.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();


	
	glPushMatrix();

	b2Vec2 pos2 = body2->GetPosition();
	float a2 = body2->GetAngle();

	glTranslatef(pos2.x, pos2.y, 0.0f);
	glRotatef(a1, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape2.m_vertices[i].x, boxshape2.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();
	
	glPushMatrix();
	glColor3f(0.8f, 0.8f, 0.8f);
	glLineWidth(1.0f);
	glBegin(GL_LINE_STRIP);
	glVertex2d(m_joint->GetAnchorA().x, m_joint->GetAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorA().x, m_joint->GetGroundAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorB().x, m_joint->GetGroundAnchorB().y);
	glVertex2d(m_joint->GetAnchorB().x, m_joint->GetAnchorB().y);
	glEnd();
	glPopMatrix();






	glPushMatrix();

	b2Vec2 pos3 = water->GetPosition();
	float32 ang3 = water->GetAngle();

	glTranslatef(pos3.x, pos3.y, 0.0f);
	glRotatef(ang3, 0.0f, 0.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(watershape.m_vertices[i].x, watershape.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();

	



	

	

	glPushMatrix();

	b2Vec2 pos4 = body3->GetPosition();
	float a4 = body3->GetAngle();

	glTranslatef(pos4.x, pos4.y, 0.0f);
	glRotatef(a4, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape3.m_vertices[i].x, boxshape3.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();


	glPushMatrix();

	b2Vec2 pos5 = body4->GetPosition();
	float a5 = body4->GetAngle();

	glTranslatef(pos5.x, pos5.y, 0.0f);
	glRotatef(a5, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape4.m_vertices[i].x, boxshape4.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();


	




	glPushMatrix();

	b2Vec2 pos6 = body5->GetPosition();
	float a6 = body5->GetAngle();

	glTranslatef(pos6.x, pos6.y, 0.0f);
	glRotatef(a6, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape5.m_vertices[i].x, boxshape5.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();


	glPushMatrix();

	b2Vec2 pos7 = body6->GetPosition();
	float a7 = body6->GetAngle();

	glTranslatef(pos7.x, pos7.y, 0.0f);
	glRotatef(a7, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8, 0.2f);

	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(boxshape6.m_vertices[i].x, boxshape6.m_vertices[i].y);

	}
	glEnd();
	glPopMatrix();


	glutSwapBuffers();

	
}


void Reshape(int _width, int _height)
{
	scr_width = _width;
	scr_height = _height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-25.0f, 25.0f, -5.0f, 45.0f);

	glViewport(0, 0, _width, _height);

	glutPostRedisplay();



}

void Dokeyboard(unsigned char key, int x, int y)
{
	int x_force = 200;
	int y_force = 200;

	switch (key)
	{
	case 'a':
		box->ApplyForce(b2Vec2(-x_force, 0), box->GetWorldCenter(), true);
		//box->ApplyTorque(2000,2000);
		break;
	case 'd':
		box->ApplyForce(b2Vec2(x_force, 0), box->GetWorldCenter(), true);
		break;
	case 'w':
		box->ApplyForce(b2Vec2(0, y_force), box->GetWorldCenter(), true);
		break;
	case 's':
		box->ApplyForce(b2Vec2(0, -y_force), box->GetWorldCenter(), true);
		break;


	default:
		break;
	}
	glutPostRedisplay();
}

int main(int argc,char** argv)
{
	glutInitWindowSize(scr_width, scr_height);
	glutInit(&argc, argv);
	glutCreateWindow("Box2D");

	Setup();

	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutTimerFunc(20, Update, 0);

	glutKeyboardFunc(Dokeyboard);

	glutMainLoop();
	
	return 0;
	

}