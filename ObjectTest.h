#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "UIMaterial.h"

//�Q�[�����

class ObjectTest :public BaseScene {
private:
public:
	//�֐�
	void deleteMem();

	ObjectTest(SceneChanger*);
	~ObjectTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};
