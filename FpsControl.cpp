#include "FpsControl.h"

FpsControl::FpsControl()
	:startTime(0) , count(0) , fps(0)
{}

void FpsControl::Update() {
	//�v���J�n�t���[���ł����
	if (count == 0) {
		startTime = GetNowCount();
	}
	if (count == N) {
		int time = GetNowCount();//���Ԃ��擾
		fps = 1000.f / ((time - startTime) / (float)N);//1�b�ɉ���X�V������
		count = 0;
		//��������X�V
		startTime = time;
	}
	count++;
}

void FpsControl::Draw() {
	SetFontSize(20);
	DrawFormatString(WIN_SIZE_X - 60, WIN_SIZE_Y - 25,COLOR_BLACK , "%.1f", fps);
}

void FpsControl::Wait() {
	//������������
	int tookTime = GetNowCount() - startTime;
	//�҂ׂ�����
	int waitTime = count*1000/FPS - tookTime;
	//�҂ׂ����Ԃ������
	if (waitTime > 0) {
		Sleep(waitTime);
	}
}