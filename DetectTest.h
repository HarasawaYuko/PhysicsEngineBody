#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "UIMaterial.h"

//ゲーム画面

class DetectTest :public BaseScene {
private:
public:
	//関数
	void deleteMem();

	DetectTest(SceneChanger*);
	~DetectTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};