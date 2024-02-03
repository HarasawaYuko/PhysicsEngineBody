#include "Circle.h"
#include "Constant.h"

Circle::Circle(const float cen_x, const float cen_y, const float r, const float v_x, const float v_y , const bool act) 
	:r(r) , Object( Vec2(v_x , v_y), CIRCLE, ( r * r * Pi), COLOR_RED, act)
{
	center.set(cen_x, cen_y);
	setBbox();
	inertiaTensor = 0.5f * mass * r * r;
	friction = 0.3f;
}

Circle::Circle(const Vec2 cen, const float r, const Vec2 v, const bool act)
	:r(r) , Object(v , CIRCLE, ( r * r * Pi), COLOR_RED, act)
{
	center = cen;
	setBbox();
	inertiaTensor = 0.5f * mass * r * r;
	friction = Constant::FRICTION;
}

float Circle::getR() const{
	return r;
}

void Circle::Draw() const{
	DrawCircleP(center.x, center.y, r, color, true);
}

void Circle::Draw(const int x_scroll, const int y_scroll) const{
	DrawCircleP(center.x + x_scroll , center.y + y_scroll, r, color, true);
	DrawCircleP(center.x + x_scroll, center.y + y_scroll, r, COLOR_BLACK, false ,2.f);
}

void Circle::DrawEdge()const {
	DrawCircleP(center.x, center.y, r, color, false ,3.f);
}

//縮尺を面積がareaになるように変更する
void Circle::changeSize(const float area) {
	float nowArea = r * r * Pi;
	float rate = area / nowArea;
	r = sqrtf(rate) * r;
	mass = r * r * Pi;
	inertiaTensor = 0.5f * mass * r * r;
}

bool Circle::isValid(const int x , const int y)const {
	//完全に画面外に出たら
	if ((center.y  + r) < 0.f || (center.x + r) < 0.f || (center.x - r) > x) {
		return false;
	}
	return true;
}

std::string Circle::toString()const {
	return "Circle";
}

void Circle::setBbox() {
	bbox.point.x = center.x - r;
	bbox.point.y =center.y - r;
	bbox.height = r * 2.f;
	bbox.width = r * 2.f;
}

void Circle::updatePos(const float step) {
	//動かない物体の場合何もしない
	if (!active) {
		return;
	}
	Vec2 deltaLinearV = (velocity * step);
	float deltaRotaV = (angle_v * step);
	if (deltaLinearV.norm() > Constant::STOP_SPEED) {
		center = center + deltaLinearV;
	}
	if (abs(deltaRotaV) > Constant::STOP_SPEED) {
		angle = angle + deltaRotaV;
	}
	//バウンディングボックスを設定
	setBbox();
}

void Circle::move(const Vec2 vec) {
	center = center + vec;
	setBbox();
}

void Circle::rotation(const float ang) {
}