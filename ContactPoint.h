#pragma once
#include "Math.h"
#include "Constraint.h"

//�Փ˓_�̏���ێ�����\����
struct ContactPoint {
	float depth;
	Vec2 pointA;//A�̏Փ˓_���[�J�����W
	Vec2 pointB;//B�̏Փ˓_���[�J�����W
	Vec2 pointA_;//A�̏Փ˓_���[�J�����W�i�ӏ�j
	Vec2 pointB_;//B�̏Փ˓_���[�J�����W�i�ӏ�j
	Vec2 normal;//�Փ˖@���x�N�g��
	Constraint constraint[2];//�S�� ��������0 ���C����1
};