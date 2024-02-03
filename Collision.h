#pragma once
#include "Object.h"
#include "ContactPoint.h"

//�Փ˃N���X

class Collision {
private:
	int contactNum;
	float friction;//���C�W��
	float e = 0.8f;
	std::vector<ContactPoint> contactPoints;

	std::pair<Object*, Object*> pair;//����Ȃ�
	uint16_t key;//����Ȃ�
	uint16_t type;//����Ȃ�
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

	void addContactPoint(const ContactPoint);//�폜�\��
};