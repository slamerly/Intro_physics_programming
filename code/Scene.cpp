//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"


/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;

	// Arene
	for (int i = 0; i < 7; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{
			float radius = 50.0f;
			float x = (i - 1) * radius * 0.25f;
			float y = (j - 2) * radius * 0.25f;

			if (i == 0)
			{
				x = (i - 2) * radius * 0.25f;
				y = (j - 2) * radius * 0.5f;
				body.position = Vec3(x, y, -7.5f);
				body.shape = new ShapeSphere(15.0f);
			}
			else if (i == 6)
			{
				x = (i) * radius * 0.25f;
				y = (j - 2) * radius * 0.5f;
				body.position = Vec3(x, y, -7.5f);
				body.shape = new ShapeSphere(15.0f);
			}
			else if (j == 0)
			{
				x = (i - 1)*radius * 0.25f;
				y = (j - 1.5) * radius * 0.5f;
				body.position = Vec3(x, y, -7.5f);
				body.shape = new ShapeSphere(15.0f);
			}
			else if (j == 4)
			{
				x = (i - 1) * radius * 0.25f;
				y = (j - 2.5) * radius * 0.5f;
				body.position = Vec3(x, y, -7.5f);
				body.shape = new ShapeSphere(15.0f);
			}
			else
			{
				body.position = Vec3(x, y, -radius);
				body.shape = new ShapeSphere(radius);
			}
			
			body.orientation = Quat(0, 0, 0, 1);
			body.inverseMass = 0.0f;
			body.elasticity = 0.99f;
			body.friction = 0.5f;
			bodies.push_back(body);
		}
	}
	/*for (int i = 0; i < 10; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{
			float radius = 100.0f;
			float x = (i - 1) * radius * 0.25f;
			float y = (j - 2) * radius * 0.25f;
			body.position = Vec3(x / 2, y / 3, -radius);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 0.0f;
			body.elasticity = 0.99f;
			body.friction = 0.5f;
			bodies.push_back(body);
		}
	}*/

	// Player balls
	/*for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float radius = 0.5f;
			float x = (i - 1) * radius * 1.5f;
			float y = (j - 1) * radius * 1.5f;
			body.position = Vec3(x, y, 5);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 1.0f;
			body.elasticity = 0.5f;
			body.friction = 0.5f;
			body.linearVelocity.Zero();
			bodies.push_back(body);
		}
	}*/

	// Cochonnet
	float radius = 0.2f;
	body.position = Vec3(0, 0, 5);
	body.orientation = Quat(0, 0, 0, 1);
	body.shape = new ShapeSphere(radius);
	body.inverseMass = 1.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	body.linearVelocity.Zero();
	body.ApplyImpulseLinear(Vec3(10, 0, 0));
	bodies.push_back(body);

}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	// Gravity
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);
	}

	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);
	
	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)alloca(sizeof(Contact) * maxContacts);
	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		
		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}

	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact), Contact::CompareContact);
	}

	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;

		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;

		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}

		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}
	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}

	//// Position update
	//for (int i = 0; i < bodies.size(); ++i) {
	//	bodies[i].Update(dt_sec);
	//}
}