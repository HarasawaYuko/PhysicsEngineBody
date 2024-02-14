#include "Mouse.h"

//�V���O���g���ւ̃A�N�Z�X��Ԃ�
Mouse* Mouse::instance() {
	static Mouse instance;
	return &instance;
}

//�}�E�X���̍X�V
void Mouse::update() {
	//�}�E�X�ʒu�̎擾
	GetMousePoint(&pos_x_ , &pos_y_);
	pos_y_ = WIN_SIZE_Y - pos_y_;
	wheel_ = GetMouseWheelRotVol();
	int nowInput = GetMouseInput();
	for (int i = 0; i < INPUT_NUM; i++) {
		if ((nowInput >> i) & 1) {
			mous_iInput_[i]++;
		}
		else {
			mous_iInput_[i] = 0;
		}
	}
}

//�N���b�N����Ă��邩���擾
bool Mouse::isClick(clickCode code) {
	return mous_iInput_[code] > 0;
}

//���N���b�N���ꂽ�u�Ԃ����擾
bool Mouse::isClickNow(clickCode code) {
	return mous_iInput_[code] == 1;
}

float Mouse::getX() {
	return (float)pos_x_;
}

float Mouse::getY() {
	return (float)pos_y_;
}

int Mouse::getWheel() {
	return wheel_;
}