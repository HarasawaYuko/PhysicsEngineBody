#pragma once

#include "Object.h"

//�ʕ�N���X
class Convex :public Object {
private:
	std::vector<Vec2> points_local_;//���[�J�����W
	std::vector<Vec2> points_world_;//���[���h���W
	int point_num_;

public:
	Convex() {}
	Convex(const std::vector<Vec2>& points , const float v_x = 0, const float v_y = 0, const float ang = 0, const float ang_v = 0, const bool act = false);
	void operator=(const Convex&);

	//�V�~�����[�V������i�߂�
	void updatePos(const float step)override;
	bool isValid(const int, const int)const override;

	//�`��
	void Draw()const override;
	void Draw(const int x_scroll, const int y_scroll)const override;
	void DrawEdge()const override;

	//�`��E�ʒu�̑���
	void move(const Vec2)override;
	void addAngle(const float);
	void changeSize(const float area) override;
	
	//�Q�b�^�[
	Vec2 getPointW(const int)const;
	Vec2 getPointL(const int)const;
	Segment getEdgeW(const int&)const;
	const std::vector<Vec2>& getPointsWorld();
	const std::vector<Vec2>& getPointsLocal();
	int getPointNum()const;

	//�Z�b�^�[
	void makeBbox()override;
	void setC(const Vec2)override;
	void recalWorldPoint();
};
