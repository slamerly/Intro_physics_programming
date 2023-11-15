//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"
#include <iostream>
#include <algorithm>


/*
========================================================================================================

Scene

========================================================================================================
*/

Scene::Scene(Application* appp): app(appp)
{
	launchPosition = Vec3(0, 0, 2);
}

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
	balls.clear();
	currentBall = nullptr;
	cptBall = 1;
	pass = false;

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
	for (int i = 0; i < 9; ++i)
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
			else if (i == 8)
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
			body.friction = 1.25f;
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
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float radius = 0.5f;
			float x = (i - 1) * radius * 1.5f;
			float y = (j - 1) * radius * 1.5f;
			body.position = Vec3(x, y, -5);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 0.0f;
			body.elasticity = 0.01f;
			body.friction = 0.95f;
			body.linearVelocity.Zero();
			bodies.push_back(body);
			balls.push_back(&body);
		}
	}

	// Cochonnet
	float radius = 0.2f;
	body.position = launchPosition;
	body.orientation = Quat(0, 0, 0, 1);
	body.shape = new ShapeSphere(radius);
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.95f;
	body.linearVelocity.Zero();
	//body.ApplyImpulseLinear(Vec3(1, 0, 0));
	bodies.push_back(body);
	balls.push_back(&body);

	currentBall = &bodies[bodies.size() - cptBall];

	//std::cout << "Throw the cochonnet." << std::endl;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	timerShoot += dt_sec;

	if (timerShoot >= outTimeShoot && !pass)
	{
		//currentBall->linearVelocity.Zero();
		//currentBall->angularVelocity.Zero();
		currentBall = &bodies[bodies.size() - cptBall];
		if (cptBall < 8)
		{
			
			currentBall->position = launchPosition;
		}
		
		if (cptBall == 8)
		{
			std::cout << "End Round." << std::endl;
			EndRound();
		}


		if (cptBall >= 2)
		{
			if (cptBall % 2 == 0)
			{
				std::cout << "Player 1 turn." << std::endl;
			}
			else
			{
				std::cout << "Player 2 turn." << std::endl;
			}
		}
		else
		{
			std::cout << "Throw the cochonnet." << std::endl;
		}
		pass = true;

		/*if (bodies[bodies.size() - 7].linearVelocity == Vec3(0, 0, 0) && cptBall == 8)
		{
			std::cout << "End Round." << std::endl;
			EndRound();
		}*/
	}

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

	//std::cout << "linear: " << bodies[bodies.size()-1].linearVelocity.x << ", " << bodies[bodies.size() - 1].linearVelocity.y << ", " << bodies[bodies.size() - 1].linearVelocity.z << std::endl;
	//std::cout << app->camLookAt.x << ", " << app->camLookAt.y << ', ' << app->camLookAt.z << std::endl;

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

void Scene::Shoot()
{
	if (timerShoot >= outTimeShoot)
	{
		if (cptBall >= 2)
		{
			currentBall->inverseMass = 2.0f;
		}
		else
		{
			currentBall->inverseMass = 1.0f;
		}

		currentBall = &bodies[bodies.size() - cptBall];
		

		ShapeSphere* shape = reinterpret_cast<ShapeSphere*>(currentBall->shape);

		Vec3 pos = app->getCamPos();
		Vec3 dir = app->m_cameraFocusPoint - pos;
		dir.Normalize();

		currentBall->orientation = Quat(dir, 0);
		//std::cout << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
		float t0, t1;

		Intersections::RaySphere(
			pos, dir,
			currentBall->GetCenterOfMassWorldSpace(),
			shape->radius,
			t0, t1
		);

		Vec3 point = pos + dir * t1;
		Vec3 power = dir * userPower;

		currentBall->ApplyImpulse(point, power);

		pass = false;
		timerShoot = 0;
		cptBall++;
	}
}

void Scene::EndRound()
{
	std::vector<float> distances;
	int pos = -1;

	Body* cocho = balls[balls.size() - 1];

	for (int i = 0; i < balls.size()-1; i++)
	{
		float dist = sqrt(pow(balls[i]->position.x - cocho->position.x, 2) 
			+ pow(balls[i]->position.y - cocho->position.y, 2) 
			+ pow(balls[i]->position.z - cocho->position.z, 2));

		distances.push_back(dist);
	}

	for (int i = 0; i < distances.size() - 1; i++)
	{
		auto minElementIterator = std::min_element(distances.begin(), distances.end());

		if (minElementIterator != distances.end()) {
			// Get the position/index of the minimum element
			int minElementIndex = std::distance(distances.begin(), minElementIterator);

			if (minElementIndex % 2 == 0)
			{
				if (pos == -1 || pos == 1)
				{
					scoreJ1++;
					pos = 1;
				}
				else
					break;
			}
			else
			{
				if (pos == -1 || pos == 2)
				{
					scoreJ2++;
					pos = 2;
				}
				else
					break;
			}

			// Remove the minimum value from the vector
			//distances.erase(minElementIterator);
			distances[minElementIndex] = 100000000;
		}
	}

	if (scoreJ1 >= 13)
	{
		std::cout << "Player 1 wins.\n\n";
	}
	else if (scoreJ2 >= 13)
	{
		std::cout << "Player 2 wins.\n\n";
	}
	else
	{
		std::cout << "Player 1: " << scoreJ1 << "\nPlayer 2: " << scoreJ2 << std::endl;
		Reset();
	}
}
