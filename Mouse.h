#pragma once
#include "include.h"

//�}�E�X���Ǘ�����N���X�i�V���O���g���j

constexpr int INPUT_NUM = 2;

//�A�N�Z�X�p�R�[�h
enum clickCode {
	LEFT_CLICK = 0,
	RIGHT_CLICK = 1,
};

class Mouse {
private:
	//�ʒu
	int posX = 0;
	int posY = 0;
	int wheel = 0;
	//�N���b�N���
	int mouseInput[INPUT_NUM] = {};
	//�R���X�g���N�^�A������Z�q��private��
	Mouse() {};
	Mouse(const Mouse&) {}
	Mouse& operator=(const Mouse&){}

public:
	static Mouse* instance();
	void update();
	bool getClick(clickCode);
	bool getClickNow(clickCode);
	float getX();
	float getY();
	int getWheel();
};