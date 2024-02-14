#include "DetectTest.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"

#include "detect.h"

#include "Pair.h"
#include "Collision.h"
#include "Convex.h"
#include "Circle.h"
#include "DEBUG.h"

//�Փˌ��m�e�X�g
static int mode = 0;

//mode =  0 �Œ蕨�̓��m�̏Փ˔���
static std::vector<Object*> objects;
static std::vector<Vec2> points;
static std::vector<Collision> collisions;
static Vec2 contactP[2];
static float r;
static Vec2 cirCenter;
static Vec2 cirEdge;
static bool makeCon = true;


DetectTest::DetectTest(SceneChanger* changer)
	:BaseScene(changer)
{}

DetectTest::~DetectTest()
{}

void DetectTest::Initialize() {
}

void DetectTest::Update() {
	//���͏�Ԃ̎擾
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	if (KeyBoard::instance()->isHitNow(KEY_INPUT_P)) {
		/*mode++;
		points.clear();*/
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_M)) {
		/*mode--;
		points.clear();*/
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Object);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Math);
	}
	Rand* rand = Rand::instance();
	switch (mode) {
	case 0:
		//�쐬�}�`�̕ύX(Switch)
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_S)) {
			makeCon = !makeCon;
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_A)) {
			cirCenter = Vec2();
			r = 0.f;
			points.clear();
			objects.clear();
			collisions.clear();
		}
		//�ʕ�̍쐬
		if (makeCon) {
			if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
				points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
			}
			if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
				Convex* con = new Convex(points);
				objects.emplace_back(con);
				objects.back()->setColor(GetColor(rand->getI(0, 150), rand->getI(0, 150), rand->getI(0, 150)));
				points.clear();
			}
		}
		//�~�쐬
		else {
			if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
				cirCenter = Vec2(Mouse::instance()->getX(), Mouse::instance()->getY());
			}
			if (Mouse::instance()->isClickNow(RIGHT_CLICK)) {
				cirEdge = Vec2(Mouse::instance()->getX(), Mouse::instance()->getY());
				r = cirCenter.distance(cirEdge);
			}
			if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN)) {
				Circle* cir = new Circle(cirCenter , r , Vec2() , false);
				objects.emplace_back(cir);
				objects.back()->setColor(GetColor(rand->getI(0 ,150), rand->getI(0, 150), rand->getI(0, 150)));
				points.clear();
			}
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_D)) {
			for (int i = 0; i < objects.size(); i++) {
				for (int j = i + 1; j < objects.size(); j++) {
					bool result = false;
					float d;
					Vec2 n;
					Vec2 coord[2];
					Vec2 coord_[2];
					switch (objects[i]->getType() | objects[j]->getType()) {
					case Combi::CIRCLE_CIRCLE:{
						//�_�E���L���X�g
						Circle* cir1 = static_cast<Circle*>(objects[0]);
						Circle* cir2 = static_cast<Circle*>(objects[1]);
					
						result = Detect::narrow(cir1, cir2, &d, &n, coord, coord_);
					}
						break;
					case Combi::CIRCLE_CONVEX: {
						Circle* cir = static_cast<Circle*>(objects[0]);
						Convex* con = static_cast<Convex*>(objects[1]);
						result = Detect::narrow(cir, con, &d, &n, coord, coord_);
					}
						break;
					case Combi::CONVEX_CONVEX: {
						Convex* con1 = static_cast<Convex*>(objects[0]);
						Convex* con2 = static_cast<Convex*>(objects[1]);
						result = Detect::narrow(con1, con2, &d, &n, coord, coord_);
					}
						break;
					}
					if (result) {
						ContactPoint cp;
						cp.depth_ = d;
						cp.pointA_ = coord[0];
						cp.pointB_ = coord[1];
						cp.pointA_edge_ = coord_[0];
						cp.pointB_edge_ = coord_[1];
						cp.normal_vec_ = n;
						Collision col = Collision(objects[i], objects[j]);
						col.addCp(cp);
						contactP[0] = LtoW(cp.pointA_, objects[0]->getCenter(), objects[0]->getAngleRad());
						contactP[1] = LtoW(cp.pointB_, objects[1]->getCenter(), objects[1]->getAngleRad());
						collisions.push_back(col);
					}
				}
			}
		}
		//�I�u�W�F�N�g��ނɂ���ă\�[�g
		std::sort(objects.begin(), objects.end(), [](const Object* a, const Object* b) {
			return (uint16_t)a->getType() < (uint16_t)b->getType();
			});
		break;
	}
}

void DetectTest::Draw() {
	SetFontSize(20);
	DrawString(640, 0, "DetectTest", COLOR_BLACK);
	DrawSegment(Segment(Vec2(0 , 300 ),Vec2(800 , 300)),COLOR_BLACK);
	DrawSegment(Segment(Vec2(400, 0), Vec2(400, 600)), COLOR_BLACK);
	Rand* rand = Rand::instance();
	int i = 0;
	switch (mode) {
	case 0:
		if (makeCon) {
			DrawStrP(Vec2(600, 50), "�ʕ�쐬(S�Ő؂�ւ�)" ,COLOR_BLUE);
		}
		else {
			DrawStrP(Vec2(600, 50), "�~�쐬(S�Ő؂�ւ�)", COLOR_BLUE);
		}
		SetFontSize(30);
		DrawString(0 , 0 , "�Փ˔���e�X�g\n" , COLOR_BLACK);
		SetFontSize(20);
		DrawString(0, 30, "LeftClick->point�ǉ� L->pointClear A->all clear enter->make \nConvex D->detect\n", COLOR_BLACK);
		//�}�`�̕`��
		for (auto obj : objects) {
			obj->DrawEdge();
		}
		if(makeCon){
			//point�̕`��
			for (auto p : points) {
				DrawPoint(p, COLOR_RED);
			}
		}
		else {
			DrawPoint(cirCenter , COLOR_RED);
			DrawPoint(cirEdge , COLOR_BLUE);
			DrawFormatString(20 , 500 , COLOR_GREEN , "r = %f" ,r);
		}
		//�Փ˂̕`��
		if (collisions.size() > 0) {
			DrawString(0 , 100 , "Detect!" ,COLOR_RED);
			Collision col = collisions[0];
			DrawStrP(col.getObj1()->getCenter() , "A" , COLOR_BLACK);
			DrawStrP(col.getObj2()->getCenter(), "B", COLOR_BLACK);
			//�Փ˓_���擾
			ContactPoint cp = col.getCp(0);
			//�ђʐ[�x��`��
			DrawFormatString(0, 150, COLOR_BLACK, "�ђʐ[�x:%f", cp.depth_);
			DrawFormatString(0, 180, COLOR_BLACK, "�@���x�N�g��:%s", cp.normal_vec_.toString().c_str());
			//�Փ˓_���擾
			Vec2 pA = LtoW(cp.pointA_edge_ , col.getObj1()->getCenter() , 0);
			Vec2 pB = LtoW(cp.pointB_edge_ , col.getObj2()->getCenter() , 0);
			DrawStrP(Vec2(550 , 510) , pA.toString() , COLOR_BLACK);
			DrawStrP(Vec2(550 , 490), pB.toString(), COLOR_BLACK);
			//�Փ˓_��`��
			DrawPoint(pA, COLOR_YELLOW);
			SetFontSize(18);
			DrawStrP(pA,"A" , COLOR_YELLOW);
			DrawPoint(pB, COLOR_YELLOW);
			DrawStrP(pB, "B" , COLOR_YELLOW);
			//�@���x�N�g���̕`��
			SetFontSize(20);
			Vec2 n = cp.normal_vec_ * cp.depth_;
			n = n + pB;
			DrawSegment(Segment(n  , pB)  , COLOR_RED);
		}
		break;
	}
}

void DetectTest::Finalize() {
	//�������̉�@
}