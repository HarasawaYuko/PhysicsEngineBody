#pragma once
#include "include.h"

//���ʕϐ����Ǘ�����static�N���X

class Share {
private:
	Share();

public:
	static int score;
	static int loadPic;

	//�v���g�^�C�v�錾
	static void initialize();
};