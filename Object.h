#pragma once
#include "Math.h"
#include "UIMaterial.h"
#include "BBox.h"

//Object�N���X�@���Z�Ώۂ̐}�`�͕K���p������
enum Type : uint8_t {
	CIRCLE = 0b0000'0001,
	CONVEX = 0b0000'0010
};

class Object {
protected:
	Vec2 center;//�d�S���W
	Vec2 velocity;//���x
	bool active = true;
	float mass;
	Type type;
	Color color;
	uint16_t id;
	bool touch;//�ڐG���Ă��邩�̃t���O
	int index;
	float friction;//���C�W��
	float e;//�����W��
	float angle_v;//�p���x�@���W�A��
	float angle;//��]�p�@���W�A��
	float inertiaTensor;//�����e���\��
	BBox bbox;//�o�E���f�B���O�{�b�N�X�i���[���h���W�j
	Object(Vec2 v , Type ,float mass = 10 ,Color color = COLOR_BLACK ,bool act = true , float ang = 0.f , float ang_v = 0.f);
public:
	Object() {}
	virtual void Draw()const = 0;
	virtual void DrawEdge()const = 0;
	virtual void Draw(const int scroll_x, const int scroll_y)const = 0;
	virtual void changeSize(const float area) = 0;
	virtual void move(const Vec2) = 0;
	virtual void rotation(const float) =  0;
	float getM();
	Vec2 getC()const;
	Vec2 getV()const;
	uint16_t getId()const;
	virtual void setC(const Vec2);
	void setV(const Vec2);
	void setAngle(const float);
	void setAngV(const float);
	void updateAngle(const float);
	float getAngle()const;
	float getAngV()const;
	bool isActive()const;
	void setMass(const float);
	void addV(const Vec2);
	void addVang(const float);
	void setId(const uint16_t);
	int getIndex()const;
	void setIndex(const int);
	void setFri(const float);
	float getFri() const;
	void setE(const float);
	float getE()const;
	Type getType()const;
	const BBox& getBbox()const;
	virtual void updatePos(const float) = 0;
	virtual bool isValid(const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y)const = 0;
	virtual std::string toString()const = 0;
	virtual void setBbox() = 0;
	virtual void setColor(Color color);
	unsigned int getColor() const;
	float getI()const;
	Vec2 getCirV(const Vec2& , const Vec2&)const;
	Vec2 getCirV(const Vec2&)const;
	bool isTouch()const;
	void setTouch(const bool);
	void operator=(const Object&);
};