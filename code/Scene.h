//
//  Scene.h
//
#pragma once
#include <vector>

#include "../Body.h"
#include "application.h"

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	//Scene() { bodies.reserve( 128 ); }
	Scene(Application* appp);
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	void Shoot();

	std::vector<Body> bodies;
	Application* app;
	float userPower = 2.5f;

private:
	Body* currentBall = nullptr;
	Vec3 launchPosition;
	float timerShoot = 5;
	float outTimeShoot = 5;
	int cptBall = 1;

	std::vector<Body*> balls;

	bool pass = false;

	int scoreJ1 = 0;
	int scoreJ2 = 0;

	void EndRound();
};

