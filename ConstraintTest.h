#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "World.h"

//�Q�[�����

class ConstraintTest :public BaseScene {
private:
	World world;
public:
	//�֐�
	void deleteMem();

	ConstraintTest(SceneChanger*);
	~ConstraintTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};