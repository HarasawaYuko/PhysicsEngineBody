#pragma once
#include "include.h"

//マウスを管理するクラス（シングルトン）

constexpr int INPUT_NUM = 2;

//アクセス用コード
enum clickCode {
	LEFT_CLICK = 0,
	RIGHT_CLICK = 1,
};

class Mouse {
private:
	//位置
	int posX = 0;
	int posY = 0;
	int wheel = 0;
	//クリック状態
	int mouseInput[INPUT_NUM] = {};
	//コンストラクタ、代入演算子をprivateに
	Mouse() {};
	Mouse(const Mouse&) {}
	Mouse& operator=(const Mouse&){}

public:
	static Mouse* instance();
	void update();
	bool getClick(clickCode);
	bool getClickNow(clickCode);
	float getX();
	float getY();
	int getWheel();
};