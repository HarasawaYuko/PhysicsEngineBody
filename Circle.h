#pragma once
#include "Object.h"

//‰~ƒNƒ‰ƒX
class Circle :public Object{
private:
	float r;//”¼Œa
public:
	Circle(const float cen_x = 0 , const float cec_y = 0 , const float r = 10.0f , const float v_x = 0, const float v_y = 0 , const bool act = true);
	Circle(const Vec2 cen , const float r , const Vec2 v , const bool act = false);
	float getR()const;
	void Draw()const override;
	void Draw(const int x_scroll, const int y_scroll)const;
	void DrawEdge()const override;
	void changeSize(const float area)override;
	bool isValid(const int , const int)const override;
	std::string toString()const override;
	void setBbox()override;
	void updatePos(const float step)override;
	void move(const Vec2)override;
	void rotation(const float)override;
};