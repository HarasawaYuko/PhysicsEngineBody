#pragma once

//定数クラス

class Constant {
public:
	static float BIAS;//めり込み解消用のバイアス
	static float GRAVITY;//重力
	static float STOP_SPEED;//停止と見なせる速度
	static int FPS;
	//摩擦係数
	static float FRICTION;
	//反発係数
	static float RESTITUTION;
};