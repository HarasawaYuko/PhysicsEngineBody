#include "Pair.h"
#include "detect.h"
#include "World.h"
#include "Solver.h"

//�����V�~�����[�V�����{��

World::World(float timeStep , const int x , const int y)
	:WORLD_SIZE_X(x) , WORLD_SIZE_Y(y) , TIME_STEP(timeStep)
{}

void World::initialize() {
	//������
	obj_num_ = 0;
	pairs_.clear();
	objects_.clear();
}

void World::physicsSimulate() {
	//�O�͂�������
	applyGravity();

	//�Փˌ��m
	detectCollision();
	
	//�S���̉���
	solveConstraints();
	
	//�ʒu�̍X�V
	updatePosition();
}

uint16_t World::addObj(Object* obj) {
	objects_.push_back(obj);
	obj_num_++;
	total_obj_num_++;
	obj->setTotalId(total_obj_num_);

	//�I�u�W�F�N�g��ނɂ���ă\�[�g
	sortObj();
	//�C���f�b�N�X�̐U�蒼��
	for (int i = 0; i < objects_.size(); i++) {
		objects_[i]->setIndex(i);
	}

	return obj->getTotalId();
}

//�͂�������
void World::applyGravity() {
	//�d�͉����x���v�Z
	float acc = -GRAVITY * TIME_STEP;
	for (Object* obj : objects_) {
		obj->addVel(Vec2(0, acc));
	}
}

//�Փˌ��m
void World::detectCollision() {
	//�u���[�h�t�F�[�Y
	detectBroard();

	//�i���[�t�F�[�Y
	for (auto it = pairs_.begin(); it != pairs_.end();) {
		const Pair& pair = *it;
		float depth;//�ђʐ[�x
		Vec2 n_vec;//�Փ˖@���x�N�g��
		Vec2 contact_coord_same[2];//�Փˍ��W
		Vec2 contact_coord_edge[2];//�Փˍ��W�i�ӏ�j
		//�Փ˂�����ނŏꍇ����
		switch (pair.getCombi()) {
		case CONVEX_CONVEX: {
			//�_�E���L���X�g
			Convex* con1 = static_cast<Convex*>(pair.getObj0());
			Convex* con2 = static_cast<Convex*>(pair.getObj1());
			if (Detect::narrow(con1, con2, &depth, &n_vec, contact_coord_same, contact_coord_edge)) {
				it++;
			}
			else {
				it = pairs_.erase(it);
				continue;
			}
		}
			break;
		case CIRCLE_CIRCLE: {
			//�_�E���L���X�g
			Circle* cir1 = static_cast<Circle*>(pair.getObj0());
			Circle* cir2 = static_cast<Circle*>(pair.getObj1());
			if (Detect::narrow(cir1, cir2, &depth, &n_vec, contact_coord_same, contact_coord_edge))
				it++;
			else {
				it = pairs_.erase(it);
				continue;
			}
		}
			break;
		case CIRCLE_CONVEX: {
			//�_�E���L���X�g
			Circle* cir = static_cast<Circle*>(pair.getObj0());
			Convex* con = static_cast<Convex*>(pair.getObj1());
			if (Detect::narrow(cir, con, &depth, &n_vec, contact_coord_same, contact_coord_edge))
				it++;
			else {
				it = pairs_.erase(it);
				continue;
			}
			break;
		}
		}
		//�Փ˂��Ă�����Փˏ����L�^
		ContactPoint cp;
		cp.depth_ = depth;
		cp.normal_vec_ = n_vec;
		cp.pointA_ = contact_coord_same[0];
		cp.pointB_ = contact_coord_same[1];
		cp.pointA_edge_ = contact_coord_edge[0];
		cp.pointB_edge_ = contact_coord_edge[1];

		pair.getCol()->addCp(cp);
	}
}


//�S���̉���
void World::solveConstraints() {
	//�S�Ẵy�A�ɂ���
	Solver::solve(this);
}

//�ʒu�̍X�V
void World::updatePosition() {
	auto itr = objects_.begin();
	bool isErase = false;//�I�u�W�F�N�g���폜���ꂽ��
	while (itr != objects_.end())
	{
		(*itr)->updatePos(TIME_STEP);
 		if (!(*itr)->isValid(WORLD_SIZE_X , WORLD_SIZE_Y))
		{
			itr = objects_.erase(itr);
			isErase = true;
			obj_num_--;
		}
		else
		{
			itr++;
		}
	}
	//�폜����������
	if (isErase) {
		//�C���f�b�N�X�̐U�蒼��
		for (int i = 0; i < objects_.size(); i++) {
			objects_[i]->setIndex(i);
		}
	}
}

//�`��@scroll �����炵�ĕ`��
void World::Draw(const int x_scroll, const int y_scroll)const {
	for (auto& obj : objects_) {
		obj->Draw(x_scroll , y_scroll);
	}
}

Object* World::searchObjById(uint16_t id)const {
	for (auto& obj : objects_) {
		if (obj->getTotalId() == id) {
			return obj;
		}
	}
	return NULL;
}

void World::finalize() {
	for (auto& obj : objects_) {
		delete obj;
	}
	objects_.clear();
	pairs_.clear();
}

void World::sortObj() {
	std::sort(objects_.begin(), objects_.end(), [](const Object* a, const Object* b) {
		return (uint16_t)a->getType() < (uint16_t)b->getType();
		});
}

//�u���[�h�t�F�[�Y
void World::detectBroard() {
	std::vector<Pair> currentPairs;//���o���ꂽ�y�A
	for (int i = 0; i < obj_num_; i++) {
		for (int j = i + 1; j < obj_num_; j++) {
			//�o�E���f�B���O�{�b�N�X�ɂ�锻��
			if (Detect::broard(objects_[i], objects_[j])) {
				//����������y�A���쐬
				currentPairs.emplace_back(objects_[i], objects_[j]);
				currentPairs.back().setType(New);
			}
		}
	}
	//�O�̃��X�g�Ɣ�r����Type��ݒ�
	for (int cur_i = 0; cur_i < currentPairs.size(); cur_i++) {
		for (int pre_i = 0; pre_i < pairs_.size(); pre_i++) {
			//������ނ����o���ꂽ��
			if (currentPairs[cur_i].getKey() == pairs_[pre_i].getKey()) {
				currentPairs[cur_i] = pairs_[pre_i];
				currentPairs[cur_i].setType(Keep);
			}
		}
	}
	//�y�A�̔z����X�V
	this->pairs_ = currentPairs;

	//�ȑO�̏Փ˓_�̏󋵂��m�F���A�X�V����
	for (Pair& pair : pairs_) {
		pair.checkContactPoints();
	}
}