#pragma once
#include "include.h"
#include "Math.h"

//�S���v�Z�ɕK�v�ȏՓˏ��

struct Constraint {
	Vec2 axis;//�S����
	float denomInv;//����
	float f;//�����S����
	float lowerF;
	float upperF;
	float accumImpulse = 0.f;//�~�ς��ꂽ�S����
};