#include "detect.h"
#include "Circle.h"
#include "Convex.h"
#include "DEBUG.h"
//プロトタイプ宣言
void projection(Vec2, Convex*, float*, float*);
bool getDepth(const float, const float, const float, const float  , float* , float*);
Vec2 getContactPoint(const Vec2& , const Segment&);
bool isCrossBbox(const BBox&, const BBox&);

bool Detect::broard(const Object* obj1 ,const Object* obj2) {
	//バウンディングボックスを取得
	const BBox& bbox1 = obj1->getBbox();
	const BBox& bbox2 = obj2->getBbox();
	if (isCrossBbox(bbox1, bbox2)) {
		return true;
	}
	return false;
}

/**ナローフェーズ****************/

//円と円の衝突判定
bool Detect::circle_circle(Object* c1 , Object* c2, float* depth, Vec2* n, Vec2* coord , Vec2* coord_) {
	//ダウンキャスト
	Circle* cir1 = static_cast<Circle*>(c1);
	Circle* cir2 = static_cast<Circle*>(c2);

	//中心間の距離を取得
	float dis = cir1->getC().distance(cir2->getC());
	float r1 = cir1->getR();
	float r2 = cir2->getR();
	if ((r1 + r2) < dis) {
		return false;
	}
	//貫通深度を取得
	*depth = dis - (r1 + r2);
	//法線ベクトル
	*n = (cir1->getC() - cir2->getC()).normalize();
	//衝突点を取得 2の円周上
	Vec2 coordWorld = cir2->getC() + (n->normalize() * r2);
	coord[0] = WtoL(coordWorld , cir1->getC() , cir1->getAngle());
	coord[1] = WtoL(coordWorld , cir2->getC() , cir2->getAngle());
	coord_[0] = WtoL(cir1->getC() + ((*n * -1) * r1) , cir1->getC() , cir1->getAngle());
	coord_[1] = coord[1];
	return true;
}

bool Detect::convex_convex(Object* c1, Object* c2, float* depth, Vec2* n, Vec2* coord , Vec2* coord_) {
	//ダウンキャスト
	Convex* con1 = static_cast<Convex*>(c1);
	Convex* con2 = static_cast<Convex*>(c2);

	//分離軸判定
	float max1, min1;//box1の投影座標の最大最小
	float max2, min2;//box2の投影座標の最大最小
	float max_depth = -INF;
	Vec2 axisMax;//分離軸候補
	//頂点→頂点のベクトルの分離軸判定
	for (int i = 0; i < con1->getPointNum(); i++) {
		for (int j = i; j < con2->getPointNum(); j++) {
			Vec2 axis = (con1->getPointW(i) - con2->getPointW(j)).normalize();
			projection(axis, con1, &min1, &max1);
			projection(axis, con2, &min2, &max2);
			float d1, d2;
			if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
				return false;
			}
			//衝突している場合、貫通深度と法線ベクトルを設定
			//assert(d1 <= 0 && d2 <= 0);
			if (max_depth < d1) {
				max_depth = d1;
				axisMax = axis;
			}
			if (max_depth < d2) {
				max_depth = d2;
				axisMax = axis * -1;
			}
		}
	}
	//辺の法線ベクトルの分離軸判定
	//convex1
	for (int i = 0; i < con1->getPointNum(); i++) {
		Vec2 axis = (con1->getPointW(i) - con1->getPointW((i + 1) % con1->getPointNum())).normalize().normal();
		projection(axis, con1, &min1, &max1);
		projection(axis, con2, &min2, &max2);
		float d1, d2;
		if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
			return false;
		}
		//衝突している場合、貫通深度と法線ベクトルを設定
		//assert(d1 <= 0 && d2 <= 0);
		if (max_depth < d1) {
			max_depth = d1;
			axisMax = axis;
		}
		if (max_depth < d2) {
			max_depth = d2;
			axisMax = axis * -1;
		}
	}
	//convex2
	for (int i = 0; i < con2->getPointNum(); i++) {
		Vec2 axis = (con2->getPointW(i) - con2->getPointW((i + 1) % con2->getPointNum())).normalize().normal();
		projection(axis, con1, &min1, &max1);
		projection(axis, con2, &min2, &max2);
		float d1, d2;
		if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
			return false;
		}
		//衝突している場合、貫通深度と法線ベクトルを設定
		//assert(d1 <= 0 && d2 <= 0);
		if (max_depth < d1) {
			max_depth = d1;
			axisMax = axis;
		}
		if (max_depth < d2) {
			max_depth = d2;
			axisMax = axis * -1;
		}
	}
	//貫通深度等の設定
	*depth = max_depth;
	*n = axisMax;


	//衝突点を取得
	float minDistance = INF;//最短距離
	bool  minA = false;
	int minPointIndex = -1;
	int minEdgeIndex = -1;
	int minPattern;
	//物体1を貫通深度より若干ずらす
	Vec2 disV = axisMax * (abs(*depth) * 2.f);//ずらすベクトル
	con1->move(disV);
	//物体1の頂点から見た最短距離
	for (int i = 0; i < con1->getPointNum(); i++) {
		for (int j = 0; j < con2->getPointNum(); j++) {
			int pattern;
			Segment edge = con2->getEdgeW(j);
			float dis = getDistance(con1->getPointW(i), edge  ,&pattern);
			if (minDistance > dis) {
				//記録
				minA = true;
				minDistance = dis;
				minPointIndex = i;
				minEdgeIndex = j;
				minPattern = pattern;
			}
		}
	}
	//物体2の頂点から見た最短距離
	for (int i = 0; i < con2->getPointNum(); i++) {
		for (int j = 0; j < con1->getPointNum(); j++) {
			int pattern;
			Segment edge = con1->getEdgeW(j);
			float dis = getDistance(con2->getPointW(i), edge ,&pattern);
			if (minDistance > dis) {
				//記録
				minA = false;
				minDistance = dis;
				minPointIndex = i;
				minEdgeIndex = j;
				minPattern = pattern;
				//printfDx("更新!-------\n");
			}
		}
	}
	
	//ずらした分戻す
	con1->move(disV * -1);

	//最短距離だった組み合わせの衝突点のローカル座標を設定
	Vec2 minPoint;
	Segment minEdge;

	if (minA) {//Aが頂点Bが辺だった場合
		//printfDx("Aが頂点\n");
		minPoint = con1->getPointW(minPointIndex);
		coord[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		coord[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		coord_[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		minEdge = con2->getEdgeW(minEdgeIndex);
		switch (minPattern) {
		case 0:
			//printfDx("パターン0\n");
			coord_[1] = WtoL(minEdge.start, con2->getC(), con2->getAngle());
			break;
		case 1:
			//printfDx("パターン1\n");
			coord_[1] = WtoL(minEdge.end, con2->getC(), con2->getAngle());
			break;
		case 2:
			//printfDx("パターン2\n");
			coord_[1] = WtoL(getContactPoint(minPoint, minEdge), con2->getC(), con2->getAngle());
			break;
		default:
			assert(false);
			break;
		}
	}
	else {
		minPoint = con2->getPointW(minPointIndex);
		coord[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		coord[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		coord_[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		minEdge = con1->getEdgeW(minEdgeIndex);
		switch (minPattern) {
		case 0:
			//始点が最近点だった場合
			coord_[0] = WtoL(minEdge.start, con1->getC(), con1->getAngle());
			break;
		case 1:
			//終点が最近点だった場合
			coord_[0] = WtoL(minEdge.end, con1->getC(), con1->getAngle());
			break;
		case 2:
			//点と直線の距離によって求める場合
			coord_[0] = WtoL(getContactPoint(minPoint, minEdge), con1->getC(), con1->getAngle());
			break;
		default:
			assert(false);
			break;
		}
	}


	//DEBUG
	Debug* debug = Debug::instance();
	debug->minPointA = minPoint;
	debug->minEdgeB = minEdge;
	return true;
}

bool Detect::circle_convex(Object* cir_, Object* con_, float* depth, Vec2* n, Vec2* coord, Vec2* coord_) {
	//ダウンキャスト
	Circle* cir = static_cast<Circle*>(cir_);
	Convex* con = static_cast<Convex*>(con_);

	//分離軸判定
	bool pointFlag = false;//頂点が衝突点かのフラグ
	int minIndex = -1;
	float minDis = FLT_MAX;
	Vec2 axis;//分離軸候補
	Vec2 cirCenter = cir->getC();//円の中心座標
	float r = cir->getR();//円の半径
	//凸包の頂点→円までの距離
	for (int i = 0; i < con->getPointNum(); i++) {
		float dis = con->getPointW(i).distance(cirCenter);
		if (minDis > dis) {
			//最短距離が更新されたら
			minDis = dis;
			pointFlag = true;
			minIndex = i;
		}
	}
	//凸包の辺→円までの距離
	int minPattern = -1;
	for (int i = 0; i < con->getPointNum(); i++) {
		int pattern;
		float dis = getDistance(cirCenter, con->getEdgeW(i), &pattern);
		if (minDis > dis) {
			minDis = dis;
			pointFlag = false;
			minIndex = i;
			minPattern = pattern;
		}
	}

	//最短距離で衝突判定
	if (minDis > r) {
		return false;
	}

	//衝突している場合、衝突情報を計算する
	Vec2 point;
	Vec2 ConToCir;
	if (pointFlag) {
		//頂点が衝突点になる場合
		point = con->getPointW(minIndex);
		ConToCir = cirCenter - point;//凸包→円のベクトル
	}
	else {
		//辺上に衝突点がある場合
		assert(minPattern == 2);
		Segment edge = con->getEdgeW(minIndex);
		point = getContactPoint(cirCenter, edge);
		ConToCir = cirCenter - point;
	}
	//法線ベクトルの設定
	*n = ConToCir.normalize();
	//貫通深度の計算
	*depth = ConToCir.norm() - r;
	//衝突点を追加
	coord[0] = WtoL(point, cir->getC(), cir->getAngle());
	coord[1] = WtoL(point, con->getC(), con->getAngle());
	coord_[0] = WtoL(cir->getC() - (ConToCir.normalize() * r) , cir->getC() ,cir->getAngle());
	coord_[1] = WtoL(point, con->getC(), con->getAngle());

	return true;
}

/**************************************************************/

//axisにbox(convex)を投影して最大と最小を返す
void projection(Vec2 axis, Convex* con, float* min, float* max) {
	float min_ = INF;
	float max_ = -INF;
	//全ての頂点を投影
	for (int i = 0; i < con->getPointNum(); i++) {
		float dot;
		dot = axis.dot(con->getPointW(i));
		min_ = min(min_, dot);
		max_ = max(max_, dot);
	}
	*min = min_;
	*max = max_;
}

//軸投影の結果、分離軸があるか判定する　交差していない(d>0)の時はtrue
bool getDepth(const float min1 , const float max1 ,const float min2 , const float max2  , float* d1 , float* d2) {
	//貫通深度を求めて、正負で判定する
	*d1 = min1 - max2;
	*d2 = min2 - max1;
	if (*d1  > 0 || *d2 > 0) {
		return true;
	}
	return false;
}

Vec2 getContactPoint(const Vec2& point , const Segment& edge) {
	//辺ベクトルの正規化
	Vec2 edgeVec =(edge.end - edge.start).normalize();
	Vec2 StoP = point - edge.start;
	//始点からの距離を求める
	float dis = StoP.dot(edgeVec);
	return edge.start + (edgeVec * dis);
}


bool isCrossBbox(const BBox& b1, const BBox& b2) {
	//x軸を調べる
	if (b1.point.x + b1.width <= b2.point.x || b2.point.x + b2.width <= b1.point.x) {
		return false;
	}
	//y軸を調べる
	if (b1.point.y + b1.height <= b2.point.y || b2.point.y + b2.height <= b1.point.y) {
		return false;
	}
	return true;
}