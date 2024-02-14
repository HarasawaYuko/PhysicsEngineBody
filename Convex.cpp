#include "Convex.h"
#include "Constant.h"

void calculateItensorArea(const std::vector<Vec2>& , float* , float* );
int partition(std::vector<std::pair<Vec2, float>>& , const int , const int);
void quick_sort(std::vector<std::pair<Vec2 , float>>& , const int  , const int);

Convex::Convex(const std::vector<Vec2> &points ,const float v_x, const float v_y, const float ang, const float ang_v, const bool act)
	:Object(Vec2() , Vec2(v_x, v_y), CONVEX, 10.f ,Constant::FRICTION , COLOR_BLACK, act, ang, ang_v)
{
	point_num_ = (int)points.size();

	//�d�S���擾
	Vec2 cen = Vec2(0, 0);
	for (auto point : points) {
		cen = cen + point;
	}
	cen = cen / (float)points.size();
	center_ = cen;
	
	//�_���d�S����\�[�g
	std::vector<std::pair<Vec2, float>> pairs_point_angle;
	//�_�Əd�S�ƂȂ��p��pair�ŕۑ�
	//todo: 0�Ԗڂ̗v�f���܂Ƃ߂��邩�m�F����
	pairs_point_angle.push_back(std::make_pair(points[0] , 0));
	for (int i = 1; i < points.size(); i++) {
		float angle = calAngle(cen, points[0], points[i]);
		pairs_point_angle.push_back(std::make_pair(points[i] , angle));
	}
	//�N�C�b�N�\�[�g
	quick_sort(pairs_point_angle , 0 , (int)pairs_point_angle.size());

	//�\�[�g���Ƀ��[�J�����W�ƃ��[���h���W��ݒ�
	for (auto pair : pairs_point_angle) {
		points_local_.push_back(pair.first - center_);
		points_world_.push_back(pair.first);
	}

	//�����e���\���Ǝ��ʂ̌v�Z
	calculateItensorArea(points_local_ , &inertiatensor_ , &mass_ );

	makeBbox();
}

//HACK:���炭�Ȃ������ǂ�
void Convex::operator=(const Convex& con) {
	this->points_local_ = con.points_local_;
	this->points_world_ = con.points_world_;
	this->point_num_ = con.point_num_;
	this->center_ = con.center_;
	this->velocity_ = con.velocity_;
	this->is_active_ = con.is_active_;
	this->mass_ = con.mass_;
	this->type_ = con.type_;
	this->color_ = con.color_;
	this->index_ = con.index_;
	this->friction_ = con.friction_;
	this->e_ = con.e_;
	this->angle_velocity_rad_ = con.angle_velocity_rad_;
	this->angle_rad_ = con.angle_rad_;
	this->inertiatensor_ = con.inertiatensor_;
	this->bbox_ = con.bbox_;
}

void Convex::updatePos(const float step) {
	//�����Ȃ����̂̏ꍇ�������Ȃ�
	if (!is_active_) {
		return;
	}

	const Vec2 deltaLinearV = (velocity_ * step);
	const float deltaRotaV = (angle_velocity_rad_ * step);
	if (deltaLinearV.norm() > Constant::STOP_SPEED) {
		center_ = center_ + deltaLinearV;
	}
	if (abs(deltaRotaV) > Constant::STOP_SPEED) {
		angle_rad_ = angle_rad_ + deltaRotaV;
	}
	recalWorldPoint();
	//�o�E���f�B���O�{�b�N�X��ݒ�
	makeBbox();
}

//�e�X�g�p�`��֐�(�ӂ�`�悷��)
void Convex::DrawEdge() const {
	for (int i = 0; i < point_num_; i++) {
		const Segment edge = getEdgeW(i);
		DrawSegment(edge, color_);
	}
}

void Convex::Draw() const {
	for (int i = 0; i < point_num_; i++) {
		const Segment edge = getEdgeW(i);
		DrawSegment(edge, color_);
	}
}

void Convex::Draw(const int x_scroll , const int y_scroll) const {
	const Vec2 scroll = Vec2((float)x_scroll , (float)y_scroll);
	for (int i = 0; i < point_num_; i++) {
		const Segment edge = getEdgeW(i);
		DrawSegment(edge.start_ + scroll , edge.end_ + scroll , COLOR_BLACK ,4.f);
		const Vec2 cen = center_ + scroll;
		const Vec2 sta = edge.start_ + scroll;
		const Vec2 end = edge.end_ + scroll;
		DrawTriP(cen, sta ,end ,color_);
		DrawSegment(cen , cen * 0.01f + sta*0.99f , color_ , 2.f);
	}
}

//Vec2�����Ƀ��[�J�����W�ƃ��[���h���W�𕽍s�ړ�����
void Convex::move(const Vec2 vec) {
	for (auto& point : points_world_) {
		point = vec + point;
	}
	center_ = center_ + vec;
	makeBbox();
}

void Convex::addAngle(const float ang) {
	angle_rad_ += ang;
	recalWorldPoint();
	makeBbox();
}

void Convex::changeSize(const float area) {
	float rate = area/mass_;
	//���[�J�����W��ύX
	for (auto& local_point : points_local_) {
		local_point = local_point * sqrtf(rate);
	}
	//���[���h���W�ɔ��f
	recalWorldPoint();

	//�����e���\���Ǝ��ʂ̌v�Z
	calculateItensorArea(points_local_ , &inertiatensor_ , &mass_ );

	makeBbox();

}

bool Convex::isValid(const int x , const int y) const {
	//�ő�l�ƍŏ��l���擾
	float max_x = FLT_MIN;
	float max_y = FLT_MIN;
	float min_x = FLT_MAX;
	for (auto& point : points_world_) {
		max_x = max(max_x , point.x_);
		max_y = max(max_y , point.y_);
		min_x = min(min_x , point.y_);
	}
	//��ʊO������
	if (max_x < 0.f || max_y < 0.f || min_x > x) {
		return false;
	}
	return true;
}

//todo: �Q�ƁAconst�Q�� �ɂ����ق����ǂ���������Ȃ�
Vec2 Convex::getPointW(const int i) const {
	if (i < 0 || i >= point_num_ ) {
		return Vec2();
	}
	return points_world_[i];
}

Vec2 Convex::getPointL(const int i) const {
	if (i < 0 || i >= point_num_) {
		return Vec2();
	}
	return points_local_[i];
}

//���[���h���W�̕ӂ�Ԃ�
Segment Convex::getEdgeW(const int& i) const {
	if (i < 0 || i >= point_num_) {
		return Segment();
	}
	return Segment(points_world_[i], points_world_[(i + 1) % point_num_]);
}

const std::vector<Vec2>& Convex::getPointsWorld() {
	return points_world_;
}

const std::vector<Vec2>& Convex::getPointsLocal() {
	return points_local_;
}

int Convex::getPointNum() const {
	return point_num_;
}

void Convex::makeBbox() {
	float max_x = -FLT_MAX;
	float min_x = FLT_MAX;
	float max_y = -FLT_MAX;
	float min_y = FLT_MAX;
	for (auto& point : points_world_) {
		max_x = max(max_x, point.x_);
		min_x = min(min_x, point.x_);
		max_y = max(max_y, point.y_);
		min_y = min(min_y, point.y_);
	}

	bbox_.origin_ = Vec2(min_x , min_y);
	bbox_.height_ = max_y - min_y;
	bbox_.width_ = max_x - min_x;
}

void Convex::setC(const Vec2 c) {
	center_ = c;
	recalWorldPoint();
}

void Convex::recalWorldPoint() {
	//���[���h���W��ݒ肵�Ȃ���
	for (int i = 0; i < point_num_; i++) {
		points_world_[i] = center_ + points_local_[i].rotationCCW(angle_rad_);
	}
}

/*************************************************/

//���_�ɑ΂���ʕ�̊����e���\�����v�Z����
//points�̓��[�J�����W
void calculateItensorArea(const std::vector<Vec2>& points , float* I , float* mass) {
	float result_I = 0.f;
	float result_mass = 0.f;
	const int point_num = (int)points.size();
	for (int i = 0; i < point_num ; i++) {
		Vec2 e0 = points[i];
		Vec2 e1 = points[(i + 1) % point_num];
		float cross = abs(e0.cross(e1));
		//�����e���\���̌�����K�p
		// 1/12 * |e_0 �~ e_1| * (e_0^2 + e_0*e_1 * e_1^2)
		result_I += (1.f / 12.f) * cross * (e0.normSquared() + e0.dot(e1) + e1.normSquared());
		//�ʐ�
		result_mass += 0.5f * cross;
	}
	*I = result_I;
	*mass = result_mass;
}

//�N�C�b�N�\�[�g�ɗp����p�[�e�[�V����
int partition(std::vector<std::pair<Vec2, float>>& v, const int s, const int e) {
	int i = s - 1;//pivot�ȉ���index
	float pivot = v[e - 1].second;
	for (int j = s; j < e;j++) {
		if (v[j].second <= pivot) {
			i++;
			auto tmp = v[j];
			v[j] = v[i];
			v[i] = tmp;
		}
	}
	return i;
}

void  quick_sort(std::vector<std::pair<Vec2, float>>& v, const int s, const int e) {
	if (s < e) {
		int p = partition(v, s, e);
		quick_sort(v, s, p );
		quick_sort(v, p + 1, e);
	}
}