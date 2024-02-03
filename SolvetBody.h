#pragma once
#include "include.h"
#include "Math.h"

//拘束を解くのに必要な情報

struct SolverBody{
public:
	SolverBody() {};
	SolverBody(const float , const float , const float);
	Vec2 deltaLinearV;//並進運動の更新される速度
	float deltaRotaV;//回転運動の更新される速度
	float angle;//姿勢
	float inertiaInv;//慣性テンソルの逆数
	float massInv;//質量の逆数

	std::string toString()const;
};