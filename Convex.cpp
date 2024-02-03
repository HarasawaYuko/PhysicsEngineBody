#include "Convex.h"
#include "Constant.h"

int partition(std::vector<std::pair<Vec2, float>>& , const int , const int);
void quick_sort(std::vector<std::pair<Vec2 , float>>& , const int  , const int);

Convex::Convex(const std::vector<Vec2> &points ,const float v_x, const float v_y, const float ang, const float ang_v, const bool act)
	:Object(Vec2(v_x , v_y), CONVEX, 10.f, COLOR_BLACK, act, ang, ang_v)
{
	//重心を取得
	Vec2 cen = Vec2(0, 0);
	for (int i = 0; i < points.size(); i++) {
		cen = cen + points[i];
	}
	cen = cen / (float)points.size();
	center = cen;
	pointNum = (int)points.size();
	//printfDx("重心%s\n" ,center.toString().c_str());
	
	//点を重心からソート
	std::vector<std::pair<Vec2, float>> pointAng;
	//点と重心となす角をpairで保存
	pointAng.push_back(std::make_pair(points[0] , 0));
	for (int i = 1; i < points.size(); i++) {
		pointAng.push_back(std::make_pair(points[i] , getTheta(cen , points[0] , points[i])));
	}
	quick_sort(pointAng , 0 , (int)pointAng.size());

	//ソート順にローカル座標とワールド座標を設定
	for (int i = 0; i < pointNum; i++) {
		pointsL.push_back(pointAng[i].first - cen);
		pointsW.push_back(pointAng[i].first);
		//printfDx("L %s\n" , pointsL[i].toString().c_str());
	}

	//慣性テンソルと質量の計算
	float I = 0.f;
	float area = 0.f;
	for (int i = 0; i < pointNum; i++) {
		Vec2 e0 = pointsL[i];
		Vec2 e1 = pointsL[(i + 1) % pointNum];
		float cross = abs(e0.cross(e1));
		I += (1.f / 12.f) * cross * (e0.squared() + e0.dot(e1) + e1.squared());
		area += 0.5f * cross;
	}
	inertiaTensor = I;
	mass = area;
	setBbox();

	friction = Constant::FRICTION;
}

void Convex::loadGraph() {
	//画像のロード、サイズの取得
}

void Convex::updatePos(const float step) {
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
	for (int i = 0; i < pointNum; i++) {
		//ワールド座標に反映
		pointsW[i] = center + pointsL[i].rotation(angle);
	}
	//バウンディングボックスを設定
	setBbox();
}

void Convex::Draw() const {
	for (int i = 0; i < pointNum; i++) {
		Segment edge = getEdgeW(i);
		DrawSegment(edge, color);
	}
}

void Convex::Draw(const int x_scroll , const int y_scroll) const {
	Vec2 scroll = Vec2((float)x_scroll , (float)y_scroll);
	for (int i = 0; i < pointNum; i++) {
		Segment edge = getEdgeW(i);
		DrawSegment(edge.start + scroll , edge.end + scroll , COLOR_BLACK ,4.f);
		Vec2 cen = center + scroll;
		Vec2 sta = edge.start + scroll;
		DrawTriP(cen, sta ,edge.end + scroll ,color);
		DrawSegment(cen , cen * 0.01f + sta*0.99f , color , 2.f);
	}
}

//テスト用描画関数(辺を描画する)
void Convex::DrawEdge() const{
	for (int i = 0; i < pointNum; i++) {
		Segment edge = getEdgeW(i);
		DrawSegment(edge , color);
	}
}

void Convex::changeSize(const float area) {
	float rate = area/mass;
	//ローカル座標を変更
	for (int i = 0; i < pointNum; i++) {
		pointsL[i] = pointsL[i] * sqrtf(rate);
	}
	for (int i = 0; i < pointNum; i++) {
		//ワールド座標に反映
		pointsW[i] = LtoW(pointsL[i], center, angle);
	}
	for (int i = 0; i < pointNum; i++) {
		//ワールド座標に反映
		pointsW[i] = LtoW(pointsL[i] , center , angle);
	}
	mass = area;
	//慣性テンソルを計算しなおす
	//慣性テンソルと質量の計算
	float I = 0.f;
	float area_ = 0.f;
	for (int i = 0; i < pointNum; i++) {
		Vec2 e0 = pointsL[i];
		Vec2 e1 = pointsL[(i + 1) % pointNum];
		float cross = abs(e0.cross(e1));
		I += (1.f / 12.f) * cross * (e0.squared() + e0.dot(e1) + e1.squared());
		area_ += 0.5f * cross;
	}
	inertiaTensor = I;
	mass = area_;
	setBbox();

}

bool Convex::isValid(const int x , const int y) const {
	//最大値と最小値を取得
	float max_x = pointsW[0].x, max_y = pointsW[0].y, min_x = pointsW[0].x;
	for (int i = 1; i < pointNum; i++) {
		if (pointsW[i].x > max_x) {
			max_x = pointsW[i].x;
		}
		if (pointsW[i].y > max_y) {
			max_y = pointsW[i].y;
		}
		if (pointsW[i].x < min_x) {
			min_x = pointsW[i].x;
		}
	}
	//画面外か判定
	if (max_x < 0.f || max_y < 0.f || min_x > x) {
		return false;
	}

	return true;
}

std::string Convex::toString()const {
	return "";
}

Vec2 Convex::getPointW(const int i) const {
	if (i < 0 || pointNum <= i) {
		return Vec2();
	}
	return pointsW[i];
}

Vec2 Convex::getPointL(const int i) const {
	if (i < 0 || pointNum <= i) {
		return Vec2();
	}
	return pointsL[i];
}

int Convex::getPointNum() const {
	return pointNum;
}

//Vec2方向にローカル座標とワールド座標を平行移動する
void Convex::move(const Vec2 vec) {
	for (int i = 0; i < pointNum; i++) {
		pointsW[i] = vec + pointsW[i];
	}
	center = center + vec;
	setBbox();
}

void Convex::rotation(const float ang) {
	angle += ang;
	setW();
	setBbox();
}

//ワールド座標の辺を返す
Segment Convex::getEdgeW(const int& i) const {
	return Segment(pointsW[i], pointsW[(i + 1) % pointNum]);
}

void Convex::setBbox() {
	float xMax = -FLT_MAX;
	float xMin = FLT_MAX;
	float yMax = -FLT_MAX;
	float yMin = FLT_MAX;
	for (int i = 0; i < pointNum; i++) {
		xMax = max(xMax , pointsW[i].x);
		xMin = min(xMin, pointsW[i].x);
		yMax = max(yMax, pointsW[i].y);
		yMin = min(yMin, pointsW[i].y);
	}
	bbox.point = Vec2(xMin , yMin);
	bbox.height = yMax - yMin;
	bbox.width = xMax - xMin;
}

void Convex::setC(const Vec2 c) {
	center = c;
	setW();
}

void Convex::setW() {
	//ワールド座標を設定しなおす
	for (int i = 0; i < pointNum; i++) {
		pointsW[i] = center + pointsL[i].rotation(angle);
	}
}

void Convex::operator=(const Convex& con) {
	this->pointsL = con.pointsL;
	this->pointsW = con.pointsW;
	this->pointNum = con.pointNum;
	this->center = con.center;
	this->velocity = con.velocity;
	this->active = con.active;
	this->mass = con.mass;
	this->type = con.type;
	this->color = con.color;
	this->index = con.index;
	this->friction = con.friction;
	this->e = con.e;
	this->angle_v = con.angle_v;
	this->angle = con.angle;
	this->inertiaTensor = con.inertiaTensor;
	this->bbox = con.bbox;
}

/*************************************************/

//クイックソートに用いるパーテーション
int partition(std::vector<std::pair<Vec2, float>>& v, const int s, const int e) {
	int i = s - 1;//pivot以下のindex
	float pivot = v[e - 1].second;
	for (int j = s; j < e;j++) {
		if (v[j].second <= pivot) {
			i++;
			auto tmp = v[j];
			v[j] = v[i];
			v[i] = tmp;
		}
	}
	return i;
}

void  quick_sort(std::vector<std::pair<Vec2, float>>& v, const int s, const int e) {
	if (s < e) {
		int p = partition(v, s, e);
		quick_sort(v, s, p );
		quick_sort(v, p + 1, e);
	}
}