#include "ConstraintTest.h"
#include "Solver.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "detect.h"
#include "Convex.h"
#include "DEBUG.h"

//衝突検知テスト
static int mode = 0;

//Convex vs Convex mode =  0
static std::vector<Object*> convexes;
static std::vector<Vec2> points;
static std::vector<Collision> collisions;
static Vec2 contactP[2];
static bool move = false;
static bool detect = false;
static Vec2 gravity = Vec2(0.f, -5.f);
static Vec2 nVec;

ConstraintTest::ConstraintTest(SceneChanger* changer)
	:BaseScene(changer)
{}

ConstraintTest::~ConstraintTest()
{}

void ConstraintTest::Initialize() {
	//三角形二つを事前に作成
	points.emplace_back(200.f , 400);
	points.emplace_back(200.f, 550);
	points.emplace_back(350.f, 400);
	Convex* con = new Convex(points , 30 , 0 , 0 , 30.f , true);
	//con->setIndex(0);
	convexes.push_back(con);
	points.clear();
	
	//points.emplace_back(350.f, 450);
	//points.emplace_back(350.f, 550);
	//points.emplace_back(450.f, 450);
	//con = new Convex(points, -35, 0, 0, 30.f, true);
	//con->setIndex(1);
	//points.clear();
	//convexes.push_back(con);

	points.emplace_back(100.f, 300);
	points.emplace_back(100.f, 0);
	points.emplace_back(600.f, 300);
	points.emplace_back(600.f, 0);
	con = new Convex(points, 0, 0, 0, 0, false);
	con->setIndex(1);
	convexes.push_back(con);
	points.clear();
}

void ConstraintTest::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	if (KeyBoard::instance()->hitNow(KEY_INPUT_P)) {
		mode++;
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_M)) {
		mode--;
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Math);
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Whole);
	}

	switch (mode) {
	case 0:
		Rand * rand = Rand::instance();
		//凸包の作成
		if (Mouse::instance()->getClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_A)) {
			points.clear();
			convexes.clear();
			collisions.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			Convex* con = new Convex(points ,0,0,0,0,true);
			convexes.push_back((Object*)con);
			convexes.back()->setColor(GetColor(rand->getI(0, 255), rand->getI(0, 255), rand->getI(0, 255)));
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_G)) {
			move = !move;
		}
		if (move) {
			//重力を加える
			for (auto con: convexes) {
				con->addV(gravity);
			}
			//衝突判定
			float d;
			Vec2 n;
			Vec2 coord[2];
			Vec2 coord_[2];
			detect = false;
			if (Detect::convex_convex(convexes[0], convexes[1], &d, &n, coord , coord_)) {
				detect = true;
				nVec = n;
				ContactPoint cp;
				cp.depth = d;
				cp.normal = n;
				cp.pointA = coord[0];
				cp.pointB = coord[1];
				Collision col = Collision(convexes[0], convexes[1]);
				col.addContactPoint(cp);
				contactP[0] = LtoW(cp.pointA, convexes[0]->getC(), convexes[0]->getAngle());
				contactP[1] = LtoW(cp.pointB, convexes[1]->getC(), convexes[1]->getAngle());
				collisions.push_back(col);
			}
			
			//拘束解消
			if(detect)Solver::solve(convexes , collisions);

			collisions.clear();

			//位置を更新
			for (auto con : convexes) {
				con->updatePos(1/30.f);
			}

		}
		break;
	}
}

void ConstraintTest::Draw() {
	SetFontSize(20);
	DrawString(640, 0, "ConstraintTest", COLOR_BLACK);
	switch (mode) {
	case 0:
		Rand * rand = Rand::instance();
		DrawString(0, 25, "凸包と凸包の拘束解消\nLeftClick->point追加 L->pointClear A->all clear enter->make \nConvex D->detect\n", COLOR_BLACK);
		//凸包の描画
		for (auto con : convexes) {
			((Convex*)con)->DrawEdge();
		}
		//pointの描画
		for (auto p : points) {
			DrawPoint(p, COLOR_RED);
		}
		if (detect) {
			DrawString(700 , 400 , "DETECT"  , COLOR_RED);
		}
		//その他描画
		DrawFormatString(10, 550, COLOR_BLUE, "n:%s", nVec.toString().c_str());
		break;
	}
}

void ConstraintTest::Finalize() {
}

//画像、音声のメモリ解放
void ConstraintTest::deleteMem() {
}