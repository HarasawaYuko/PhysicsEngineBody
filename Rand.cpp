#include "Rand.h"

//�V���O���g���ւ̃A�N�Z�X��Ԃ�
Rand* Rand::instance() {
	static Rand instance;
	return &instance;
}

float Rand::getF(const int min, const int max) {
	return (float)(min + mt() % ((int)max + 1 - (int)min));
}

int Rand::getI(const int min, const int max) {
	return (int)(min + mt() % ((int)max + 1 - (int)min));
}