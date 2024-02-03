#include "BBox.h"
#include "UIMaterial.h"

void BBox::Draw() const{
	DrawSegment(point , point + Vec2(0 , height) ,COLOR_BLACK);
	DrawSegment(point, point + Vec2(width, 0), COLOR_BLACK);
	DrawSegment(point + Vec2(0 , height), point + Vec2(width, height), COLOR_BLACK);
	DrawSegment(point + Vec2(width , 0), point + Vec2(width, height), COLOR_BLACK);
}