#pragma once
#include "include.h"

//KeyBoard�N���X(�V���O���g��)

class KeyBoard {
private:
	int key_state_[256];

	//�R���X�g���N�^�A�R�s�[�R���X�g���N�^�A������Z�q��private
	KeyBoard(){};
	KeyBoard(const KeyBoard&) {};
	KeyBoard& operator=(const KeyBoard&) {};
public:
	static KeyBoard* instance();

	void update();

	bool isHitNow(const int);
	bool isPush(const int);
	int getFrame(const int);
};