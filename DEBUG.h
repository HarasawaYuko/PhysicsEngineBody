#pragma once
#include "Math.h"
//�f�o�b�O�p�̕ϐ�

class Debug {
private:
	//�R���X�g���N�^�A������Z�q��private��
	Debug() {};
	Debug(const Debug&) {}
	Debug& operator=(const Debug&) {}

public:
	static Debug* instance();
	//convex detect�p
	Segment minEdgeB;
	Vec2 minPointA;
};