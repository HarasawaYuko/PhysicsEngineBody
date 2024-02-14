#include "Math.h"

Vec2::Vec2()
	:x_(0) , y_(0)
{}

Vec2::Vec2(const float x , const float y) 
	:x_(x) , y_(y)
{}

void Vec2::set(const float x , const float y) {
	this->x_ = x;
	this->y_ = y;
}

float Vec2::dot(const Vec2 vec) const{
	return x_ * vec.x_ + y_ * vec.y_;
}

float Vec2::cross(const Vec2 vec)const{
	return x_ * vec.y_ - y_ * vec.x_;
}

float Vec2::norm() const {
	return (float)sqrt(x_ * x_ + y_ * y_);
}

float Vec2::normSquared()const {
	return (float)(x_ * x_ + y_ * y_);
}

float Vec2::distance(const Vec2 vec) const{
	Vec2 sub = *this - vec;
	return sub.norm();
}

Vec2 Vec2::normalize()const {
	float length = norm();
	return Vec2(x_ / length, y_ / length);
}

Vec2 Vec2::rotationCCW(const float & ang)const {
	float x = (float)(x_*cos(ang) - y_*sin(ang));
	float y = (float)(x_*sin(ang) + y_*cos(ang));
	return Vec2(x , y);
}

//�@���x�N�g����Ԃ�
//�����v�����90�x��]����
Vec2 Vec2::normalCCW()const {
	return Vec2(-y_ ,x_);
}

Vec2 Vec2::operator+(const Vec2 vec) const {
	return Vec2(x_ + vec.x_, y_ + vec.y_);
}

Vec2 Vec2::operator+(const float val)const {
	return Vec2(x_ + val, y_ + val);
}

Vec2 Vec2::operator-(const Vec2 vec) const {
	return Vec2(x_ - vec.x_, y_ - vec.y_);
}

Vec2 Vec2::operator-(const float val) const {
	return Vec2(x_ - val, y_ - val);
}

Vec2 Vec2::operator*(const float k) const {
	return Vec2(x_ * k, y_ * k);
}

Vec2 Vec2::operator/(const float k) const {
	return Vec2(x_ / k, y_ / k);
}

std::string Vec2::toString() const
{
	std::string str;
	char tmp[255];
	str += "(";
	sprintf_s(tmp, "%f", x_);
	str += std::string(tmp);
	str += ",";
	sprintf_s(tmp, "%f", y_);
	str += std::string(tmp);
	str += ")";
	return str;
}

/**�s��̎���**/

//row ��@column �s
Matrix::Matrix(const int row, const int column) 
	:row_num_(row) , column_num_(column)
{
	matrix_.resize(row);
	for (auto& row : matrix_) {
		row.resize(column);
	}
}

//�񎟌��x�N�g������s��ɃL���X�g
Matrix::Matrix(const Vec2& vec) 
	:row_num_(1) , column_num_(2)
{
	matrix_.resize(row_num_);
	matrix_[0].resize(2);

	matrix_[0][0] = vec.x_;
	matrix_[0][1] = vec.y_;
}

void Matrix::identity() {
	//�����s��̂�
	if (row_num_ != column_num_) {
		return;
	}
	for (int it_r = 0; it_r < row_num_; it_r++) {
		for (int it_c = 0; it_c < column_num_; it_c++) {
			if (it_r == it_c) {
				matrix_[it_r][it_c] = 1.f;
			}
			else {
				matrix_[it_r][it_c] = 0.f;
			}
		}
	}
}

//�s��� right...�E��
Matrix Matrix::product(const Matrix& right) const{
	//�s��̌`���m�F����
	if (this->row_num_ != right.column_num_) {
		return Matrix(0 ,0);
	}
	Matrix result = Matrix(right.row_num_ ,this->column_num_ );
	for (int c = 0; c < this->column_num_; c++) {
		for (int r = 0; r < right.row_num_; r++) {
			float element = 0.f;
			for (int r2 = 0; r2 < this-> row_num_ ; r2++) {
				element += this->matrix_[r2][c] * right.matrix_[r][r2];
			}
			result.matrix_[r][c] = element;
		}
	}
	return result;
}

//�]�u�s���Ԃ�
Matrix Matrix::trans()const {
	Matrix result = Matrix(column_num_ ,row_num_);
	for (int r = 0; r < row_num_; r++) {
		for (int c = 0; c < column_num_; c++) {
			result.matrix_[c][r] = matrix_[r][c];
		}
	}
	return result;
}

std::string Matrix::toString() const {
	std::string str;
	for (int i = 0; i < column_num_ ; i++) {
		for (int j = 0; j < row_num_; j++) {
			char tmp[255];
			sprintf_s(tmp ,"%f", matrix_[j][i]);
			str += std::string(tmp);
			str += ",";
		}
		str += "\n";
	}
	return str;
}

Matrix Matrix::operator+(const float k)const {
	Matrix result = Matrix(row_num_ , column_num_);
	for (int r = 0; r < row_num_;r++) {
		for (int c = 0; c < column_num_;c++) {
			result.matrix_[r][c] = matrix_[r][c] + k;
		}
	}
	return result;
}

Matrix Matrix::operator+(const Matrix& mat)const {
	Matrix result = Matrix(row_num_, column_num_);
	for (int r = 0; r < row_num_; r++) {
		for (int c = 0; c < column_num_; c++) {
			result.matrix_[r][c] = matrix_[r][c] + mat.matrix_[r][c];
		}
	}
	return result;
}

Matrix Matrix::operator*(const float k)const {
	Matrix result = Matrix(row_num_ , column_num_);
	for (int r = 0; r < row_num_; r++) {
		for (int c = 0; c < column_num_; c++) {
			result.matrix_[r][c] = matrix_[r][c] * k;
		}
	}
	return result;
}

Matrix Matrix::operator*(const Matrix& mat)const {
	Matrix result = Matrix(row_num_, column_num_);
	for (int r = 0; r < row_num_; r++) {
		for (int c = 0; c < column_num_; c++) {
			result.matrix_[r][c] = matrix_[r][c] * mat.matrix_[r][c];
		}
	}
	return result;
}

Matrix Matrix::operator-(const Matrix& mat)const {
	Matrix result = Matrix(row_num_, column_num_);
	for (int r = 0; r < row_num_; r++) {
		for (int c = 0; c < column_num_; c++) {
			result.matrix_[r][c] = matrix_[r][c] - mat.matrix_[r][c];
		}
	}
	return result;
}

Segment::Segment(const Vec2 s , const Vec2 e) 
	:start_(s) , end_(e)
{}

//�x���@�����W�A���ɕϊ�����
float RadToDeg(const float ang) {
	return (ang * kPi) / 180.f;
}

//���W�A����x���@�ɕϊ�����
float DegToRad(const float rad) {
	return rad * (180.f / kPi);
}

//�����Ɠ_�̈ʒu�֌W�𒲂ׂ�
//0...�n�_���ɓ_������  1...�I�_���ɓ_������  2...�����̗̈���ɓ_������
int posRelationSegPoint(const Vec2& point , const Segment& seg)
{
	const Vec2 StoE = seg.end_ - seg.start_;
	const Vec2 StoP = point - seg.start_;
	const Vec2 EtoP = point - seg.end_;
	if (StoE.dot(StoP) < 0.f) {
		return 0;
	}
	if ((StoE * -1.f).dot(EtoP) < 0.f) {
		return 1;
	}
	return 2;
}

//�_�ƒ����̋���
float distanceLinePoint(const Vec2& point , const Segment& line) {
	const Vec2 StoP = point - line.start_;
	const Vec2 StoE = line.end_ - line.start_;
	return abs((StoP).cross(StoE)) / StoE.norm();
}

/*
* �_�Ɛ����̋���
*/
float distanceSegPoint(const Vec2& point, const Segment& seg ) {
	switch (posRelationSegPoint(point , seg)) {
	case 0:
		return point.distance(seg.start_);
	case 1:
		return point.distance(seg.end_);
	case 2:
		return distanceLinePoint(point , seg);
	default:
		//posRelationSegPoint()�ɃG���[������͂�
		return -1.f;
	}
}

//cen->p0 �� cen->p1 �̂Ȃ��p�����߂�(�����v���)
float calAngle(const Vec2& cen , const Vec2& p0 , const Vec2& p1) {
	const Vec2 vec1 = (p0 - cen).normalize();
	const Vec2 vec2 = (p1 - cen).normalize();
	//180 < x < 360�̎�
	if (vec1.cross(vec2) < 0) {
		return 2.f * kPi - (float)acos(vec1.dot(vec2));
	}
	//0<= x <= 180 �̎�
	else {
		//�����덷�p�ɃN�����v����
		float dot = clamp(vec1.dot(vec2) , -1.f , 1.0f);
		return (float)acos(dot);
	}
}

//���[�J�����W�����[���h���W�ɕϊ�
Vec2 LtoW(const Vec2& target_local, const Vec2& cen_world , float ang) {
	return target_local.rotationCCW(ang) + cen_world;
}

//���[���h���W�����[�J�����W�ɕϊ�
Vec2 WtoL(const Vec2 &target_world , const Vec2& center_world , float ang) {
	return (target_world - center_world).rotationCCW(-ang);
}

//�~�^���̑��x
Vec2 rotationVelocity(const Vec2& r , const float omega) {
	//�傫�����擾 v = r��
	float length = r.norm() * omega;
	return (r.normalCCW().normalize()) * length;
}

float clamp(const float value, const float lower , const float upper) {
	if (value < lower) return lower;
	if (value > upper) return upper;
	return value;
}