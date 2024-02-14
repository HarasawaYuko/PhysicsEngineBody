#include "Collision.h"

Collision::Collision() 
	:contact_num_(0)
{}

Collision::Collision(Object* obj1 , Object* obj2 )
	:contact_num_(0) , obj1_(obj1) , obj2_(obj2)
{}

//�����Ɨǂ��������@�����肻��
void Collision::addCp(ContactPoint add_point) {
	//�����̏Փ˓_�Ɠ����_�ł���΁A������X�V����
	const float kSamePoint = Constant::JUDGE_SAME_POINT;//�����_�ƌ��Ȃ���͈�
	int exist_point_index = -1;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint& exist_point = contactpoints_[i];
		//�ՓːV�K�̓_�Ɣ�r
		const float distance_A = (add_point.pointA_ - exist_point.pointA_).norm();
		const float distance_B = (add_point.pointB_ - exist_point.pointB_).norm();
		const float dot_nvec = add_point.normal_vec_.dot(exist_point.normal_vec_);
		if (distance_A < kSamePoint && distance_B < kSamePoint && dot_nvec > 0.99f ) {
			exist_point_index = i;
			break;
		}
	}

	//�����̏Փ˓_�Ɠ���_�����o���ꂽ��
	if (exist_point_index != -1) {
		//�����̏Փ˓_��V�������ɍX�V����
		contactpoints_[exist_point_index].depth_ = add_point.depth_;
		contactpoints_[exist_point_index].normal_vec_ = add_point.normal_vec_;
		contactpoints_[exist_point_index].pointA_ = add_point.pointA_;
		contactpoints_[exist_point_index].pointB_ = add_point.pointB_;
		return;
	}

	//�V�K�Փ˓_�̏ꍇ
	//�܂��ǉ��ł���ꍇ
	if (contact_num_ < 2) {
		contactpoints_.push_back(add_point);
		contact_num_++;
		return;
	}

	//���ɓ����ꍇ
	contactpoints_.push_back(add_point);
	 contact_num_++;
	//�ђʐ[�x���ł��[�����͕̂K���̗p����
	int left_points_indexes[2];//�̗p����Փ˓_�̃C���f�b�N�X
	ContactPoint deepest_point;
	float deep_max = FLT_MAX;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint cp = contactpoints_[i];
		if ( cp.depth_ < deep_max ) {
			deep_max = cp.depth_;
			deepest_point = cp;
			left_points_indexes[0] = i;
		}
	}

	//�m�肵���_����ł������_��I������
	float distace_max = FLT_MIN;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint cp = contactpoints_[i];
		const float distance_with_deepest = cp.pointA_.distance(deepest_point.pointA_);
		if (distance_with_deepest > distace_max) {
			distace_max = distance_with_deepest;
			left_points_indexes[1] = i;
		}
	}

	for (int i = 0; i < contact_num_;i++) {
		if (i == left_points_indexes[0] || i == left_points_indexes[1]) {
			continue;
		}
		else {
			contactpoints_.erase(contactpoints_.begin() + i);
			contact_num_--;
			break;
		}
	}
	return;
}

void Collision::deleteCp(const int index) {
	assert(0 <= index && index < contact_num_);
	contactpoints_.erase(contactpoints_.begin() + index);
	contact_num_--;
}

Object* Collision::getObj1()const {
	return obj1_;
}

Object* Collision::getObj2()const {
	return obj2_;
}

float Collision::getE()const {
	return e_;
}

float Collision::getFri()const {
	return friction_;
}

int Collision::getContactNum()const {
	return contact_num_;
}

ContactPoint& Collision::getCp(const int i) {
	return contactpoints_[i];
}

void Collision::setFri(const float fri) {
	friction_ = fri;
}

void Collision::setE(const float e) {
	e_ = e;
}