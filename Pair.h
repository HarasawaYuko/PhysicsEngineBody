#pragma once
#include "Math.h"
#include "Object.h"
#include "Collision.h"

/*
*�y�A�N���X
* �u���[�h�t�F�[�Y����������܂ł͎g�p���Ȃ�
*/
enum Kind : uint8_t {
	CIRCLE_CIRCLE = 0b0000'0001,
	CIRCLE_CONVEX = 0b0000'0011,
	CONVEX_CONVEX = 0b0000'0010
};

enum PairType {
	New,
	Keep
};

class Pair {
private:
	Object* obj[2];//�I�u�W�F�N�g�ւ̃A�N�Z�X
	Kind kind;//�I�u�W�F�N�g�̑g�ݍ��킹
	PairType type;//�V�K�Ɍ��������y�A��
	uint32_t key;//���j�[�N�ȃL�[
	Collision* collision;//�Փˏ��
public:
	Pair(Object* , Object*);
	Kind getKind()const;
	PairType getType()const;
	uint32_t getKey()const;
	void setType(PairType);
	Object* getObj0()const;
	Object* getObj1()const;
	Collision* getCol()const;
	void refreshCp();//contactPoint�����t���b�V������
	std::string toString()const;
};