#pragma once

#include "Task.h"
#include "SceneChanger.h"

//�V�[���̊��N���X

class BaseScene : public Task {
protected:
	//�V�[���ύX�p
	SceneChanger* m_sceneChanger;

public:
	BaseScene(SceneChanger*);
	virtual ~BaseScene();
	virtual void Initialize() override;
	virtual void Finalize() override;
	virtual void Update() override;
	virtual void Draw() override;
};