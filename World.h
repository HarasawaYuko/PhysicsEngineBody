#pragma once
#include <random>

#include "Object.h"
#include "Pair.h"
#include "Constant.h"

//�����V�~�����[�V�����N���X

class World {
private:
	const float GRAVITY = Constant::GRAVITY;//�d�͉����x
	const float TIME_STEP;
	//�V�~�����[�V�������[���h�̃T�C�Y
	const int WORLD_SIZE_X;
	const int WORLD_SIZE_Y;

	uint16_t total_obj_num_;//�ʎZ�Œǉ������I�u�W�F�N�g�̐��@65536�ȏ�ɂȂ������U���Z�b�g����K�v����
	int obj_num_;//world���ɂ���I�u�W�F�N�g�̐�

	void applyGravity();
	void detectCollision();
	void solveConstraints();
	void updatePosition();

	void detectBroard();

	void sortObj();
	
public:
	//�R���X�g���N�^
	World(float timeStep = 1.f/(float)FPS , const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y);
	void initialize();
	void finalize();
	void physicsSimulate();
	uint16_t addObj(Object*);
	Object* searchObjById(uint16_t id)const;
	void Draw(const int, const int) const;

	std::vector<Object*> objects_;
	std::vector<Pair> pairs_;
};