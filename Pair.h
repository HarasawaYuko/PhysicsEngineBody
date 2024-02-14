#pragma once
#include "Math.h"
#include "Object.h"
#include "Collision.h"

/*
*�y�A�N���X
* �u���[�h�t�F�[�Y�Ō��������g�ݍ��킹��ۑ����Ă���
*/

//�Փ˕��̂̑g�ݍ��킹
enum Combi : uint8_t {
	CIRCLE_CIRCLE = 0b0000'0001,
	CIRCLE_CONVEX = 0b0000'0011,
	CONVEX_CONVEX = 0b0000'0010
};

//�����̃^�C�~���O
//New ���̃^�C���X�e�b�v�Ō��m���ꂽ
//Keep �O�̃^�C���X�e�b�v�Ō��m����Ă���
enum PairType {
	New,
	Keep
};

class Pair {
private:
	Object* objects_[2];//�I�u�W�F�N�g�ւ̃A�N�Z�X
	Combi combi_kind_;//�I�u�W�F�N�g�̑g�ݍ��킹
	PairType type_;//�V�K�Ɍ��������y�A��
	uint32_t key_;//���j�[�N�ȃL�[
	Collision* collision_;//�Փˏ��

public:
	Pair( Object* , Object*);

	//�V�~�����[�V����
	void checkContactPoints();

	//�Q�b�^�[
	const Combi& getCombi()const;
	const PairType& getType()const;
	uint32_t getKey()const;
	Object* getObj0()const;
	Object* getObj1()const;
	Collision* getCol()const;
	//�Z�b�^�[
	void setType(PairType);
	
	std::string toString()const;
};