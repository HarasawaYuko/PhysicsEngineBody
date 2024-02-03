#include "ObjectTest.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "detect.h"
#include "Convex.h"
#include "DEBUG.h"

//�}�`�e�X�g
static int mode = 0;

//�ʕ�e�X�g mode = 0
static std::vector<Vec2> points;
static Convex* convex;
static bool make = false;

//�ʕ��]
static bool rotation = false;
static Vec2 vel;
static float angV = Pi/2;
static float dot;

ObjectTest::ObjectTest(SceneChanger* changer)
	:BaseScene(changer)
{}

ObjectTest::~ObjectTest()
{}

void ObjectTest::Initialize() {
}

void ObjectTest::Update() {
	//���͏�Ԃ̎擾
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	if (KeyBoard::instance()->hitNow(KEY_INPUT_P)) {
		mode++;
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_M)) {
		mode--;
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_Result);
	}
	if (KeyBoard::instance()->hitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Detect);
	}

	Rand* rand = Rand::instance();
	switch (mode) {
	case 0:
		//�ʕ�̍쐬
		if (Mouse::instance()->getClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_A)) {
			delete convex;
			make = false;
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			delete convex;
			convex = new Convex(points);
			convex->setColor(GetColor(rand->getI(0, 255), rand->getI(0, 255), rand->getI(0, 255)));
			points.clear();
			make = true;
		}
		break;
	case 1:
		if (Mouse::instance()->getClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_A)) {
			delete convex;
			make = false;
			rotation = false;
			points.clear();
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			delete convex;
			convex = new Convex(points,0,0,0,0,true);
			convex->setColor(GetColor(rand->getI(0, 255), rand->getI(0, 255), rand->getI(0, 255)));
			points.clear();
			((Object*)convex)->setAngV(angV);
			make = true;
		}
		if (KeyBoard::instance()->hitNow(KEY_INPUT_R)) {
			rotation = !rotation;
		}
		if (rotation) {
			convex->updatePos(0.1f);
			vel = convex->getCirV(convex->getPointL(0));
			dot = (convex->getPointW(0)-convex->getC()).dot(vel);
		}
		break;
	}
}

void ObjectTest::Draw() {
	SetFontSize(20);
	DrawString(640, 0, "ObjectTest", COLOR_BLACK);
	DrawString(600 , 22 ,"P->mode��i�߂� M->mode��߂�" , COLOR_BLACK);
	switch (mode) {
	case 0:
		DrawString(0, 0, "�ʕ�Ɠʕ�̐���\nLeftClick->point�ǉ� L->pointClear A->all clear enter->make \nConvex D->detect\n", COLOR_BLACK);
		//point
		for (auto p : points) {
			DrawPoint(p , COLOR_RED);
		}
		//�ʕ�̕`��
		if (make) {
			convex->DrawEdge();
			DrawFormatString(0, 80, COLOR_RED, "�����e���\��:%f", convex->getI());
			DrawFormatString(0, 110, COLOR_RED, "����:%f", convex->getM());
		}
		break;
	case 1:
		DrawString(0, 0, "�ʕ�̉�]\nLeftClick->point�ǉ� L->pointClear A->all clear enter->make \nConvex R->��]\n", COLOR_BLACK);
		//point
		for (auto p : points) {
			DrawPoint(p, COLOR_RED);
		}
		//�ʕ�̕`��
		if (make) {
			convex->DrawEdge();
			DrawFormatString(0, 80, COLOR_RED, "�p���x rad:%f fre:%f", convex->getAngV() , getDegree(convex->getAngV()));
			DrawFormatString(0, 110, COLOR_RED, "���� %f", dot);
			DrawFormatString(0, 140, COLOR_BLACK, "�傫�� %f", vel.norm());
			//printfDx("%f \n", convex->getPointL(0).toString().c_str());
			//printfDx("vel %s\n" , vel.toString().c_str());
			//���x�x�N�g���̕`��
			Segment vSeg = Segment(convex->getPointW(0) , (convex->getPointW(0)+vel));
			DrawSegment(vSeg, COLOR_RED);
			DrawSegment(Segment(convex->getC() , convex->getPointW(0)), COLOR_BLACK);
		}
		break;
	}

}

void ObjectTest::Finalize() {
}

//�摜�A�����̃��������
void ObjectTest::deleteMem() {
}