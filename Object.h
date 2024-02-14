#pragma once
#include "BBox.h"
#include "Constant.h"
#include "Math.h"
#include "UIMaterial.h"

/*
Object�N���X�@���Z�Ώۂ̐}�`�͕K���p������
*/

//�`���\���񋓑�
//�g�ݍ��킹��\�����邽�߂�one hot�\��������
enum Type : uint8_t {
	CIRCLE = 0b0000'0001,
	CONVEX = 0b0000'0010
};

class Object {
protected:
	//�^���A����
	Vec2 center_;
	Vec2 velocity_;
	float mass_;
	float angle_rad_;//��]�p�@���W�A��
	float angle_velocity_rad_;//�p���x�@���W�A��
	float inertiatensor_;//�����e���\��
	float friction_;//���C�W��
	float e_;//�����W��

	//�Ǘ��p
	Type type_;
	uint16_t id_total_;
	int index_;
	BBox bbox_;//�o�E���f�B���O�{�b�N�X�i���[���h���W�j

	//���̑�
	bool is_active_ = true;
	Color color_;

	Object(Vec2 cen ,Vec2 v , Type ,float mass = 10 , float fri = Constant::FRICTION , Color color = COLOR_BLACK ,bool act = true , float ang = 0.f , float ang_v = 0.f);

public:
	Object() {}

	//�`��
	virtual void Draw()const = 0;
	virtual void Draw(const int scroll_x, const int scroll_y)const = 0;
	virtual void DrawEdge()const = 0;

	//�V�~�����[�V����
	virtual void updatePos(const float) = 0;
	virtual bool isValid(const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y)const = 0;


	//�ړ��A�ό`
	virtual void changeSize(const float area) = 0;
	virtual void move(const Vec2) = 0;
	void rotation(const float);
	void addVel(const Vec2);
	void addVelAng(const float);
	virtual void makeBbox() = 0;

	Vec2 rotationVelocityVec(const Vec2&)const;

	//�Q�b�^�[
	const Vec2& getCenter()const;
	const Vec2& getVelocity()const;
	const float& getMass();
	const float& getAngleRad()const;
	const float& getAngVelRad()const;
	const float& getIne()const;
	const float& getFri() const;
	const float& getE()const;
	const Type& getType()const;
	const uint16_t& getTotalId()const;
	const int& getIndex()const;
	const BBox& getBbox()const;
	const bool& isActive()const;
	const Color& getColor()const;

	//�Z�b�^�[
	virtual void setC(const Vec2);
	void setV(const Vec2);
	void setMass(const float);
	void setAngleRad(const float);
	void setAngVRad(const float);
	void setFri(const float);
	void setE(const float);
	void setTotalId(const uint16_t);
	void setIndex(const int);
	virtual void setColor(const Color color);

	std::string toString()const;//todo:virtual�ɂ��������ǂ�
};