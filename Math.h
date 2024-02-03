#pragma once
#include "include.h"

const float Pi = 3.141592f;
const float INF = 10000000000.0f;

//二次元ベクトル
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
	
	float dot(const Vec2)const;//内積
	float cross(const Vec2)const;//外積
	float norm()const;//大きさ
	float squared()const;
	float distance(const Vec2)const;//２点間の距離
	Vec2 normalize()const;//正規化
	Vec2 rotation(const float &)const;//回転
	Vec2 normal()const;
	std::string toString()const;
};

struct Matrix {
	int row;//列
	int column;//行
	std::vector<std::vector<float>> matrix;//[row][column]
	
	Matrix(const int , const int);
	Matrix(const Vec2&);
	float get(const int , const int)const;
	std::string toString()const;
	void identity();//単位行列にする
	Matrix product(const Matrix&) const;//行列積
	Matrix trans()const;//転置行列を返す
	
	Matrix operator+(const float)const;
	Matrix operator+(const Matrix&)const;
	Matrix operator*(const float)const;
	Matrix operator*(const Matrix&)const;
	Matrix operator-(const Matrix&)const;
};

//線分
struct Segment {
public:
	Segment(){}
	Segment(const Vec2 , const Vec2);
	Vec2 start;
	Vec2 end;
};

//ラジアンと度数法の変換
float getRad(const float);
float getDegree(const float);

//点と直線の距離を求める
float getDistanceLine(const Vec2& , const Segment &);
float getDistance(const Vec2&, const Segment&);
float getDistance(const Vec2&, const Segment & , int*);

//なす角を取得
float getTheta(const Vec2 & , const Vec2 & , const Vec2 &);

Vec2 LtoW(const Vec2& local,const Vec2& worldCen,const float ang);
Vec2 WtoL(const Vec2&, const Vec2& ,const float);

//円運動の速度を取得する v = rω
Vec2 getVang(const Vec2& ,const float);

//値をクランプする
float clamp(const float , const float , const float);