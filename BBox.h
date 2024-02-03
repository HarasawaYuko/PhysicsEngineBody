#pragma once
#include "include.h"
#include "Math.h"

//バウンディングボックス

struct BBox {
	float height;//高さ
	float width;//幅
	Vec2 point;//左下の点

	void Draw()const;
};