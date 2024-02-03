#pragma once

//�V�[���p�񋓌^
enum Scene{
	Scene_Menu,
	Scene_Game,
	Scene_Result,
	Scene_TEST_Object,
	Scene_TEST_Detect,
	Scene_TEST_Math,
	Scene_TEST_Constraint,
	Scene_TEST_Whole,
	Scene_Fin,
	Scene_None
};

//�V�[���ύX�p�C���^�[�t�F�[�X
class SceneChanger {
public:
	virtual void ChangeScene(Scene next) = 0;
};