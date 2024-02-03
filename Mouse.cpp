#include "Mouse.h"

//�V���O���g���ւ̃A�N�Z�X��Ԃ�
Mouse* Mouse::instance() {
	static Mouse instance;
	return &instance;
}

//�}�E�X���̍X�V
void Mouse::update() {
	//�}�E�X�ʒu�̎擾
	GetMousePoint(&posX , &posY);
	posY = WIN_SIZE_Y - posY;
	wheel = GetMouseWheelRotVol();
	int nowInput = GetMouseInput();
	for (int i = 0; i < INPUT_NUM; i++) {
		if ((nowInput >> i) & 1) {
			mouseInput[i]++;
		}
		else {
			mouseInput[i] = 0;
		}
	}
}

//�N���b�N����Ă��邩���擾
bool Mouse::getClick(clickCode code) {
	return mouseInput[code] > 0;
}

//���N���b�N���ꂽ�u�Ԃ����擾
bool Mouse::getClickNow(clickCode code) {
	return mouseInput[code] == 1;
}

float Mouse::getX() {
	return (float)posX;
}

float Mouse::getY() {
	return (float)posY;
}

int Mouse::getWheel() {
	return wheel;
}