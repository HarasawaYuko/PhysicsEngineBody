#pragma once

//���ۃN���X

class Task {
public:
	//�������Ȃ��Ă��ǂ�
	virtual void Initialize(){}
	virtual void Finalize(){}
	//�������K�{
	virtual void Update() = 0;
	virtual void Draw() = 0;
};