#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "World.h"
#include "UIMaterial.h"
#include "Circle.h"

#include "Pair.h"

//ゲーム画面

class WholeTest :public BaseScene {
private:
	World world;
public:
	//関数
	void deleteMem();

	WholeTest(SceneChanger*);
	~WholeTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};