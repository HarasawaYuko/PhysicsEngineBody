#include "KeyBoard.h"

KeyBoard* KeyBoard::instance() {
	static KeyBoard instance;
	return &instance;
}

void KeyBoard::update() {
	//ó‘Ô‚ğæ“¾
	char tmpKey[256];
	GetHitKeyStateAll(tmpKey);
	for (int i = 0 ; i < 256; i++) {
		if (tmpKey[i] == 1) {
			keyState[i]++;
		}
		else {
			keyState[i] = 0;
		}
	}
}

bool KeyBoard::hitNow(const int key) {
	if (keyState[key] == 1) {
		return true;
	}
	return false;
}

bool KeyBoard::getState(const int key) {
	if (keyState[key] > 0) {
		return true;
	}
	return false;
}

int KeyBoard::getFrame(const int key) {
	return keyState[key];
}