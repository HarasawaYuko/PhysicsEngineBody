#include "Collision.h"

Collision::Collision(Object* obj1, Object* obj2) 
	:pair(std::make_pair(obj1 , obj2))
	,type(obj1->getType() | obj2 -> getType())
	,contact_num_(0)
{
	uint16_t id1 = (uint16_t)obj1->getTotalId();
	uint16_t id2 = (uint16_t)obj2->getTotalId();
	key = (id1 << 8) | id2;
}

Collision::Collision() 
	:contact_num_(0)
{
}

void Collision::addCp(ContactPoint cp) {
	//既存の衝突点と同じ点であれば、それを更新する
	const float SAME_POINT = 0.1f;//同じ点と見なせる範囲
	int existIndex = -1;//同じ点だった時のindex
	for (int i = 0; i < contact_num_; i++) {
		ContactPoint& exist = contactpoints_[i];
		//衝突新規の点と比較
		float diffA = (cp.pointA_ - exist.pointA_).norm();
		float diffB = (cp.pointB_ - exist.pointB_).norm();
		float dot = cp.normal_vec_.dot(exist.normal_vec_);//法線ベクトルの角度が同じか
		if (diffA < SAME_POINT && diffB < SAME_POINT && dot > 0.99f ) {
			existIndex = i;
			break;
		}
	}

	//新規衝突点の場合
	if (existIndex == -1) {
		//まだ追加できる場合
		if (contact_num_ < 2) {
			contactpoints_.push_back(cp);
			contact_num_++;
			return;
		}
		//既に二つある場合
		else {
			contactpoints_.push_back(cp);
			//貫通深度が最も深いものは必ず採用する
			float maxDepth = FLT_MAX;
			int deepestIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (maxDepth > contactpoints_[i].depth_) {
					maxDepth = contactpoints_[i].depth_;
					deepestIndex = i;
				}
			}
			ContactPoint deepest = contactpoints_[deepestIndex];
			contactpoints_.erase(contactpoints_.begin() + deepestIndex);
			//確定点から最も離れてる点を採用する
			if (contactpoints_[0].pointA_.distance(deepest.pointA_) < contactpoints_[1].pointA_.distance(deepest.pointA_)) {
				contactpoints_.erase(contactpoints_.begin() + 0);
			}
			else {
				contactpoints_.erase(contactpoints_.begin() + 1);
			}
			contactpoints_.push_back(deepest);
			//衝突点が複数ある場合は距離が最も遠いものを選択
			float dis = contactpoints_[0].pointA_.distance(contactpoints_[1].pointA_);
			float dis0 = contactpoints_[0].pointA_.distance(cp.pointA_);
			float dis1 = contactpoints_[1].pointA_.distance(cp.pointA_);
			int index;
			float disNew;
			if (dis0 < dis1) {
				index = 1;
				disNew = dis1;
			}
			else {
				index = 0;
				disNew = dis0;
			}
			if (dis < disNew) {
				contactpoints_[index] = cp;
			}
			return;
		}
	}
	//既存の衝突点の場合
	else {
		//新しい情報に更新する
		contactpoints_[existIndex].depth_ = cp.depth_;
		contactpoints_[existIndex].normal_vec_ = cp.normal_vec_;
		contactpoints_[existIndex].pointA_ = cp.pointA_;
		contactpoints_[existIndex].pointB_ = cp.pointB_;
		//printfDx("accume--- %f\n", contactPoints[existIndex].constraint->accumImpulse);
		return;
	}
}

void Collision::deleteCp(const int index) {
	assert(0 <= index && index < contact_num_);
	contactpoints_.erase(contactpoints_.begin() + index);
	contact_num_--;
}

uint16_t Collision::getKey() const{
	return key;
}

uint16_t Collision::getType() const {
	return type;
}

Object* Collision::getObj1()const {
	return pair.first;
}

Object* Collision::getObj2()const {
	return pair.second;
}

float Collision::getE()const {
	return e_;
}

int Collision::getContactNum()const {
	return contact_num_;
}

void Collision::Draw() const{
	for (auto cp : contactpoints_) {
		//ローカル座標→ワールド座標に戻す
		Vec2 pA = LtoW(cp.pointA_ , pair.first->getCenter() ,pair.first->getAngleRad() );
		Vec2 pB = LtoW(cp.pointB_, pair.second->getCenter(), pair.second->getAngleRad());
		//Vec2 pB = LtoW();
		//衝突点の描画
		DrawCircle((int)pA.x_ , (int)(WIN_SIZE_Y - pA.y_) , 3 , COLOR_RED);
		DrawCircle((int)pB.x_ , (int)(WIN_SIZE_Y - pB.y_) , 3 , COLOR_RED);
		//法線ベクトルの描画
		Vec2 nVec = pA + (cp.normal_vec_ * 100);
		DrawLine((int)pA.x_, (int)(WIN_SIZE_Y - pA.y_,nVec.x_) , (int)(WIN_SIZE_Y - nVec.y_)  , COLOR_RED , 3);
	}
}

void Collision::setFri(const float fri) {
	friction_ = fri;
}

float Collision::getFri()const {
	return friction_;
}

ContactPoint& Collision::getCp(const int i){
	assert(0 <= i && i < contact_num_);
	return contactpoints_[i];
}

//削除予定
void Collision::addContactPoint(const ContactPoint cp) {
	contactpoints_[0] = cp;
}