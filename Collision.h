#pragma once
#include "Object.h"
#include "ContactPoint.h"

//衝突クラス

class Collision {
private:
	int contact_num_;
	float friction_;//摩擦係数
	float e_ = 0.8f;
	std::vector<ContactPoint> contactpoints_;

	std::pair<Object*, Object*> pair;//いらない
	uint16_t key;//いらない
	uint16_t type;//いらない
public:
	void addCp(ContactPoint);
	void deleteCp(const int);
	Collision(Object* , Object*);
	Collision();
	uint16_t getKey()const;
	uint16_t getType()const;
	Object* getObj1()const;
	Object* getObj2()const;
	float getE()const;
	int getContactNum()const;
	void Draw()const;
	void setFri(const float );
	float getFri()const;
	ContactPoint& getCp(const int);

	void addContactPoint(const ContactPoint);//削除予定
};