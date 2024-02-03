#include "SceneMgr.h"
#include "DetectTest.h"
#include "MathTest.h"
#include "ObjectTest.h"
#include "ConstraintTest.h"
#include "WholeTest.h"

SceneMgr::SceneMgr() {
	m_scene = (BaseScene*)new WholeTest(this);
}

void SceneMgr::Initialize() {
	m_next_scene = Scene_None;
	m_scene->Initialize();
}

void SceneMgr::Finalize() {
	m_scene->Finalize();
}

void SceneMgr::Update() {
	if (m_next_scene != Scene_None) {
		//���݂̃V�[���̏I������
		m_scene->Finalize();
		//�������̉�@
		delete m_scene;
		switch (m_next_scene) {
			break;
		case Scene_TEST_Object:
			m_scene = (BaseScene*) new ObjectTest(this);
			break;
		case Scene_TEST_Detect:
			m_scene = (BaseScene*) new DetectTest(this);
			break;
		case Scene_TEST_Math:
			m_scene = (BaseScene*) new MathTest(this);
			break;
		case Scene_TEST_Constraint:
			m_scene = (BaseScene*) new ConstraintTest(this);
			break;
		case Scene_TEST_Whole:
			m_scene = (BaseScene*) new WholeTest(this);
			break;
		}
		m_next_scene = Scene_None;
		m_scene->Initialize();
	}
	m_scene->Update();
}

void SceneMgr::Draw() {
	m_scene->Draw();
}

void SceneMgr::ChangeScene(Scene next) {
	if (next == Scene_Fin) fin = true;
	m_next_scene = next;
}