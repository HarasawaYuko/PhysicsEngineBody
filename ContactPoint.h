#pragma once
#include "Math.h"
#include "Constraint.h"

//�Փ˓_�̏���ێ�����\����
struct ContactPoint {
	float depth_;
	Vec2 pointA_;//A�̏Փ˓_���[�J�����W
	Vec2 pointB_;//B�̏Փ˓_���[�J�����W
	Vec2 pointA_edge_;//A�̏Փ˓_���[�J�����W�i�ӏ�j
	Vec2 pointB_edge_;//B�̏Փ˓_���[�J�����W�i�ӏ�j
	Vec2 normal_vec_;//�Փ˖@���x�N�g��
	Constraint constraint_[2];//�S�� ��������0 ���C����1
};