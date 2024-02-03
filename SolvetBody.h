#pragma once
#include "include.h"
#include "Math.h"

//�S���������̂ɕK�v�ȏ��

struct SolverBody{
public:
	SolverBody() {};
	SolverBody(const float , const float , const float);
	Vec2 deltaLinearV;//���i�^���̍X�V����鑬�x
	float deltaRotaV;//��]�^���̍X�V����鑬�x
	float angle;//�p��
	float inertiaInv;//�����e���\���̋t��
	float massInv;//���ʂ̋t��

	std::string toString()const;
};