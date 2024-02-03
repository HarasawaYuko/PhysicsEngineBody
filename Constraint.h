#pragma once
#include "include.h"
#include "Math.h"

//S‘©ŒvZ‚É•K—v‚ÈÕ“Ëî•ñ

struct Constraint {
	Vec2 axis;//S‘©²
	float denomInv;//•ª•ê
	float f;//‰ŠúS‘©—Í
	float lowerF;
	float upperF;
	float accumImpulse = 0.f;//’~Ï‚³‚ê‚½S‘©—Í
};