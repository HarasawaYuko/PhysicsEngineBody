#pragma once
#include "include.h"
#include "Math.h"

//�S���v�Z�ɕK�v�ȏՓˏ��

struct Constraint {
	Vec2 axis_;//�S����
	float f_denominator_;//����
	float f_;//�����S����
	float lower_f_;
	float upper_f_;
	float accum_impulse_ = 0.f;//�~�ς��ꂽ�S����
};