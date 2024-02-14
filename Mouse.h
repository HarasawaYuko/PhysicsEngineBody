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
	int pos_x_ = 0;
	int pos_y_ = 0;
	int wheel_ = 0;
	//�N���b�N���
	int mous_iInput_[INPUT_NUM] = {};
	//�R���X�g���N�^�A������Z�q��private��
	Mouse() {};
	Mouse(const Mouse&) {}
	Mouse& operator=(const Mouse&){}

public:
	static Mouse* instance();
	void update();

	bool isClick(clickCode);
	bool isClickNow(clickCode);
	float getX();
	float getY();
	int getWheel();
};