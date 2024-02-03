#pragma once
#include "include.h"

const float Pi = 3.141592f;
const float INF = 10000000000.0f;

//�񎟌��x�N�g��
struct Vec2 {
private:
public:
	float x = 0;
	float y = 0;
	Vec2();
	Vec2(const float x,const float y);

	void set(const float, const float);

	Vec2 operator+(const Vec2)const;
	Vec2 operator+(const float)const;
	Vec2 operator-(const Vec2)const;
	Vec2 operator-(const float)const;
	Vec2 operator*(const float)const;
	Vec2 operator/(const float)const;
	
	float dot(const Vec2)const;//����
	float cross(const Vec2)const;//�O��
	float norm()const;//�傫��
	float squared()const;
	float distance(const Vec2)const;//�Q�_�Ԃ̋���
	Vec2 normalize()const;//���K��
	Vec2 rotation(const float &)const;//��]
	Vec2 normal()const;
	std::string toString()const;
};

struct Matrix {
	int row;//��
	int column;//�s
	std::vector<std::vector<float>> matrix;//[row][column]
	
	Matrix(const int , const int);
	Matrix(const Vec2&);
	float get(const int , const int)const;
	std::string toString()const;
	void identity();//�P�ʍs��ɂ���
	Matrix product(const Matrix&) const;//�s���
	Matrix trans()const;//�]�u�s���Ԃ�
	
	Matrix operator+(const float)const;
	Matrix operator+(const Matrix&)const;
	Matrix operator*(const float)const;
	Matrix operator*(const Matrix&)const;
	Matrix operator-(const Matrix&)const;
};

//����
struct Segment {
public:
	Segment(){}
	Segment(const Vec2 , const Vec2);
	Vec2 start;
	Vec2 end;
};

//���W�A���Ɠx���@�̕ϊ�
float getRad(const float);
float getDegree(const float);

//�_�ƒ����̋��������߂�
float getDistanceLine(const Vec2& , const Segment &);
float getDistance(const Vec2&, const Segment&);
float getDistance(const Vec2&, const Segment & , int*);

//�Ȃ��p���擾
float getTheta(const Vec2 & , const Vec2 & , const Vec2 &);

Vec2 LtoW(const Vec2& local,const Vec2& worldCen,const float ang);
Vec2 WtoL(const Vec2&, const Vec2& ,const float);

//�~�^���̑��x���擾���� v = r��
Vec2 getVang(const Vec2& ,const float);

//�l���N�����v����
float clamp(const float , const float , const float);