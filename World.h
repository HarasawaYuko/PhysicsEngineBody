#pragma once
#include <random>

#include "Object.h"
#include "Pair.h"

//�����V�~�����[�V�����N���X

class World {
private:
	const float gravity = 200.8f;//�d�͉����x
	float TIME_STEP;
	uint16_t totalNum;//�ʎZ�Œǉ������I�u�W�F�N�g�̐��@65536�ȏ�ɂȂ������U���Z�b�g����K�v����
	int objNum;//world���ɂ���I�u�W�F�N�g�̐�
	int SIZE_X;
	int SIZE_Y;

	void applyForce();
	void detectCollision();
	void solveConstraints();
	void integrate();
public:
	World(float timeStep = 1.f/(float)FPS , const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y);
	std::vector<Object*> objects;
	std::vector<Pair> pairs;
	void initialize();
	void physicsSimulate();
	uint16_t add(Object*);
	void Draw(const int , const int) const;
	Object* getObj(uint16_t id)const;
	void finalize();
};