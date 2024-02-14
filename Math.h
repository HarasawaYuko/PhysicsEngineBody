#pragma once

#include <corecrt_math_defines.h>

#include "include.h"

const float INF = FLT_MAX;
const float kPi = (float)M_PI;

//�񎟌��x�N�g��
struct Vec2 {
public:
	float x_ = 0;
	float y_ = 0;

	Vec2();
	Vec2(const float x,const float y);

	void set(const float, const float);
	
	//���Z
	float dot(const Vec2)const;//����
	float cross(const Vec2)const;//�O��
	float norm()const;//�傫��
	float normSquared()const;
	float distance(const Vec2)const;//�Q�_�Ԃ̋���
	Vec2 normalize()const;//���K��
	Vec2 rotationCCW(const float &)const;//��]
	Vec2 normalCCW()const;

	//���Z�q�I�[�o�[���[�h
	Vec2 operator+(const Vec2)const;
	Vec2 operator+(const float)const;
	Vec2 operator-(const Vec2)const;
	Vec2 operator-(const float)const;
	Vec2 operator*(const float)const;
	Vec2 operator/(const float)const;

	std::string toString()const;
};

struct Matrix {
	int row_num_;//��
	int column_num_;//�s
	std::vector<std::vector<float>> matrix_;//[row][column]
	
	Matrix(const int , const int);
	Matrix(const Vec2&);
	
	//�s�񉉎Z
	void identity();//�P�ʍs��ɂ���
	Matrix product(const Matrix&) const;//�s���
	Matrix trans()const;//�]�u�s���Ԃ�
	
	//���Z�q�I�[�o�[���[�h
	Matrix operator+(const float)const;
	Matrix operator+(const Matrix&)const;
	Matrix operator*(const float)const;
	Matrix operator*(const Matrix&)const;
	Matrix operator-(const Matrix&)const;

	std::string toString()const;
};

//����
struct Segment {
public:
	Segment(){}
	Segment(const Vec2 , const Vec2);

	Vec2 start_;
	Vec2 end_;
};

//���W�A���Ɠx���@�̕ϊ�
float RadToDeg(const float);
float DegToRad(const float);

//�_�ƒ����̋��������߂�
int posRelationSegPoint(const Vec2& , const Segment &);
float distanceLinePoint(const Vec2& , const Segment &);
float distanceSegPoint(const Vec2&, const Segment&);

//�Ȃ��p���擾
float calAngle(const Vec2 & , const Vec2 & , const Vec2 &);

Vec2 LtoW(const Vec2& local,const Vec2& worldCen,const float ang);
Vec2 WtoL(const Vec2&, const Vec2& ,const float);

//�~�^���̑��x���擾���� v = r��
Vec2 rotationVelocity(const Vec2& ,const float);

//�l���N�����v����
float clamp(const float , const float , const float);