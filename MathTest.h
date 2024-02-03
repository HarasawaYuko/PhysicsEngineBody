#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "World.h"
#include "UIMaterial.h"
#include "Circle.h"

#include "Pair.h"

//�Q�[�����

class MathTest :public BaseScene {
private:
	World world;
public:
	//�֐�
	void deleteMem();

	MathTest(SceneChanger*);
	~MathTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};