#pragma once
#include "include.h"
#include "UIMaterial.h"

class FpsControl {
private:
	int startTime;//����J�n����
	int count;//�J�E���^
	float fps;//���ۂ�fps
	static const int N = 60;//�T���v����
public:
	FpsControl();
	void Update();
	void Draw();
	void Wait();
};