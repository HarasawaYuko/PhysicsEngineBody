#include "detect.h"
#include "Circle.h"
#include "Convex.h"
#include "DEBUG.h"
//�v���g�^�C�v�錾
void projection(Vec2, Convex*, float*, float*);
bool getDepth(const float, const float, const float, const float  , float* , float*);
Vec2 getContactPoint(const Vec2& , const Segment&);
bool isCrossBbox(const BBox&, const BBox&);

bool Detect::broard(const Object* obj1 ,const Object* obj2) {
	//�o�E���f�B���O�{�b�N�X���擾
	const BBox& bbox1 = obj1->getBbox();
	const BBox& bbox2 = obj2->getBbox();
	if (isCrossBbox(bbox1, bbox2)) {
		return true;
	}
	return false;
}

/**�i���[�t�F�[�Y****************/

//�~�Ɖ~�̏Փ˔���
bool Detect::circle_circle(Object* c1 , Object* c2, float* depth, Vec2* n, Vec2* coord , Vec2* coord_) {
	//�_�E���L���X�g
	Circle* cir1 = static_cast<Circle*>(c1);
	Circle* cir2 = static_cast<Circle*>(c2);

	//���S�Ԃ̋������擾
	float dis = cir1->getC().distance(cir2->getC());
	float r1 = cir1->getR();
	float r2 = cir2->getR();
	if ((r1 + r2) < dis) {
		return false;
	}
	//�ђʐ[�x���擾
	*depth = dis - (r1 + r2);
	//�@���x�N�g��
	*n = (cir1->getC() - cir2->getC()).normalize();
	//�Փ˓_���擾 2�̉~����
	Vec2 coordWorld = cir2->getC() + (n->normalize() * r2);
	coord[0] = WtoL(coordWorld , cir1->getC() , cir1->getAngle());
	coord[1] = WtoL(coordWorld , cir2->getC() , cir2->getAngle());
	coord_[0] = WtoL(cir1->getC() + ((*n * -1) * r1) , cir1->getC() , cir1->getAngle());
	coord_[1] = coord[1];
	return true;
}

bool Detect::convex_convex(Object* c1, Object* c2, float* depth, Vec2* n, Vec2* coord , Vec2* coord_) {
	//�_�E���L���X�g
	Convex* con1 = static_cast<Convex*>(c1);
	Convex* con2 = static_cast<Convex*>(c2);

	//����������
	float max1, min1;//box1�̓��e���W�̍ő�ŏ�
	float max2, min2;//box2�̓��e���W�̍ő�ŏ�
	float max_depth = -INF;
	Vec2 axisMax;//���������
	//���_�����_�̃x�N�g���̕���������
	for (int i = 0; i < con1->getPointNum(); i++) {
		for (int j = i; j < con2->getPointNum(); j++) {
			Vec2 axis = (con1->getPointW(i) - con2->getPointW(j)).normalize();
			projection(axis, con1, &min1, &max1);
			projection(axis, con2, &min2, &max2);
			float d1, d2;
			if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
				return false;
			}
			//�Փ˂��Ă���ꍇ�A�ђʐ[�x�Ɩ@���x�N�g����ݒ�
			//assert(d1 <= 0 && d2 <= 0);
			if (max_depth < d1) {
				max_depth = d1;
				axisMax = axis;
			}
			if (max_depth < d2) {
				max_depth = d2;
				axisMax = axis * -1;
			}
		}
	}
	//�ӂ̖@���x�N�g���̕���������
	//convex1
	for (int i = 0; i < con1->getPointNum(); i++) {
		Vec2 axis = (con1->getPointW(i) - con1->getPointW((i + 1) % con1->getPointNum())).normalize().normal();
		projection(axis, con1, &min1, &max1);
		projection(axis, con2, &min2, &max2);
		float d1, d2;
		if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
			return false;
		}
		//�Փ˂��Ă���ꍇ�A�ђʐ[�x�Ɩ@���x�N�g����ݒ�
		//assert(d1 <= 0 && d2 <= 0);
		if (max_depth < d1) {
			max_depth = d1;
			axisMax = axis;
		}
		if (max_depth < d2) {
			max_depth = d2;
			axisMax = axis * -1;
		}
	}
	//convex2
	for (int i = 0; i < con2->getPointNum(); i++) {
		Vec2 axis = (con2->getPointW(i) - con2->getPointW((i + 1) % con2->getPointNum())).normalize().normal();
		projection(axis, con1, &min1, &max1);
		projection(axis, con2, &min2, &max2);
		float d1, d2;
		if (getDepth(min1, max1, min2, max2, &d1, &d2)) {
			return false;
		}
		//�Փ˂��Ă���ꍇ�A�ђʐ[�x�Ɩ@���x�N�g����ݒ�
		//assert(d1 <= 0 && d2 <= 0);
		if (max_depth < d1) {
			max_depth = d1;
			axisMax = axis;
		}
		if (max_depth < d2) {
			max_depth = d2;
			axisMax = axis * -1;
		}
	}
	//�ђʐ[�x���̐ݒ�
	*depth = max_depth;
	*n = axisMax;


	//�Փ˓_���擾
	float minDistance = INF;//�ŒZ����
	bool  minA = false;
	int minPointIndex = -1;
	int minEdgeIndex = -1;
	int minPattern;
	//����1���ђʐ[�x���኱���炷
	Vec2 disV = axisMax * (abs(*depth) * 2.f);//���炷�x�N�g��
	con1->move(disV);
	//����1�̒��_���猩���ŒZ����
	for (int i = 0; i < con1->getPointNum(); i++) {
		for (int j = 0; j < con2->getPointNum(); j++) {
			int pattern;
			Segment edge = con2->getEdgeW(j);
			float dis = getDistance(con1->getPointW(i), edge  ,&pattern);
			if (minDistance > dis) {
				//�L�^
				minA = true;
				minDistance = dis;
				minPointIndex = i;
				minEdgeIndex = j;
				minPattern = pattern;
			}
		}
	}
	//����2�̒��_���猩���ŒZ����
	for (int i = 0; i < con2->getPointNum(); i++) {
		for (int j = 0; j < con1->getPointNum(); j++) {
			int pattern;
			Segment edge = con1->getEdgeW(j);
			float dis = getDistance(con2->getPointW(i), edge ,&pattern);
			if (minDistance > dis) {
				//�L�^
				minA = false;
				minDistance = dis;
				minPointIndex = i;
				minEdgeIndex = j;
				minPattern = pattern;
				//printfDx("�X�V!-------\n");
			}
		}
	}
	
	//���炵�����߂�
	con1->move(disV * -1);

	//�ŒZ�����������g�ݍ��킹�̏Փ˓_�̃��[�J�����W��ݒ�
	Vec2 minPoint;
	Segment minEdge;

	if (minA) {//A�����_B���ӂ������ꍇ
		//printfDx("A�����_\n");
		minPoint = con1->getPointW(minPointIndex);
		coord[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		coord[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		coord_[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		minEdge = con2->getEdgeW(minEdgeIndex);
		switch (minPattern) {
		case 0:
			//printfDx("�p�^�[��0\n");
			coord_[1] = WtoL(minEdge.start, con2->getC(), con2->getAngle());
			break;
		case 1:
			//printfDx("�p�^�[��1\n");
			coord_[1] = WtoL(minEdge.end, con2->getC(), con2->getAngle());
			break;
		case 2:
			//printfDx("�p�^�[��2\n");
			coord_[1] = WtoL(getContactPoint(minPoint, minEdge), con2->getC(), con2->getAngle());
			break;
		default:
			assert(false);
			break;
		}
	}
	else {
		minPoint = con2->getPointW(minPointIndex);
		coord[0] = WtoL(minPoint, con1->getC(), con1->getAngle());
		coord[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		coord_[1] = WtoL(minPoint, con2->getC(), con2->getAngle());
		minEdge = con1->getEdgeW(minEdgeIndex);
		switch (minPattern) {
		case 0:
			//�n�_���ŋߓ_�������ꍇ
			coord_[0] = WtoL(minEdge.start, con1->getC(), con1->getAngle());
			break;
		case 1:
			//�I�_���ŋߓ_�������ꍇ
			coord_[0] = WtoL(minEdge.end, con1->getC(), con1->getAngle());
			break;
		case 2:
			//�_�ƒ����̋����ɂ���ċ��߂�ꍇ
			coord_[0] = WtoL(getContactPoint(minPoint, minEdge), con1->getC(), con1->getAngle());
			break;
		default:
			assert(false);
			break;
		}
	}


	//DEBUG
	Debug* debug = Debug::instance();
	debug->minPointA = minPoint;
	debug->minEdgeB = minEdge;
	return true;
}

bool Detect::circle_convex(Object* cir_, Object* con_, float* depth, Vec2* n, Vec2* coord, Vec2* coord_) {
	//�_�E���L���X�g
	Circle* cir = static_cast<Circle*>(cir_);
	Convex* con = static_cast<Convex*>(con_);

	//����������
	bool pointFlag = false;//���_���Փ˓_���̃t���O
	int minIndex = -1;
	float minDis = FLT_MAX;
	Vec2 axis;//���������
	Vec2 cirCenter = cir->getC();//�~�̒��S���W
	float r = cir->getR();//�~�̔��a
	//�ʕ�̒��_���~�܂ł̋���
	for (int i = 0; i < con->getPointNum(); i++) {
		float dis = con->getPointW(i).distance(cirCenter);
		if (minDis > dis) {
			//�ŒZ�������X�V���ꂽ��
			minDis = dis;
			pointFlag = true;
			minIndex = i;
		}
	}
	//�ʕ�̕Ӂ��~�܂ł̋���
	int minPattern = -1;
	for (int i = 0; i < con->getPointNum(); i++) {
		int pattern;
		float dis = getDistance(cirCenter, con->getEdgeW(i), &pattern);
		if (minDis > dis) {
			minDis = dis;
			pointFlag = false;
			minIndex = i;
			minPattern = pattern;
		}
	}

	//�ŒZ�����ŏՓ˔���
	if (minDis > r) {
		return false;
	}

	//�Փ˂��Ă���ꍇ�A�Փˏ����v�Z����
	Vec2 point;
	Vec2 ConToCir;
	if (pointFlag) {
		//���_���Փ˓_�ɂȂ�ꍇ
		point = con->getPointW(minIndex);
		ConToCir = cirCenter - point;//�ʕ�~�̃x�N�g��
	}
	else {
		//�ӏ�ɏՓ˓_������ꍇ
		assert(minPattern == 2);
		Segment edge = con->getEdgeW(minIndex);
		point = getContactPoint(cirCenter, edge);
		ConToCir = cirCenter - point;
	}
	//�@���x�N�g���̐ݒ�
	*n = ConToCir.normalize();
	//�ђʐ[�x�̌v�Z
	*depth = ConToCir.norm() - r;
	//�Փ˓_��ǉ�
	coord[0] = WtoL(point, cir->getC(), cir->getAngle());
	coord[1] = WtoL(point, con->getC(), con->getAngle());
	coord_[0] = WtoL(cir->getC() - (ConToCir.normalize() * r) , cir->getC() ,cir->getAngle());
	coord_[1] = WtoL(point, con->getC(), con->getAngle());

	return true;
}

/**************************************************************/

//axis��box(convex)�𓊉e���čő�ƍŏ���Ԃ�
void projection(Vec2 axis, Convex* con, float* min, float* max) {
	float min_ = INF;
	float max_ = -INF;
	//�S�Ă̒��_�𓊉e
	for (int i = 0; i < con->getPointNum(); i++) {
		float dot;
		dot = axis.dot(con->getPointW(i));
		min_ = min(min_, dot);
		max_ = max(max_, dot);
	}
	*min = min_;
	*max = max_;
}

//�����e�̌��ʁA�����������邩���肷��@�������Ă��Ȃ�(d>0)�̎���true
bool getDepth(const float min1 , const float max1 ,const float min2 , const float max2  , float* d1 , float* d2) {
	//�ђʐ[�x�����߂āA�����Ŕ��肷��
	*d1 = min1 - max2;
	*d2 = min2 - max1;
	if (*d1  > 0 || *d2 > 0) {
		return true;
	}
	return false;
}

Vec2 getContactPoint(const Vec2& point , const Segment& edge) {
	//�Ӄx�N�g���̐��K��
	Vec2 edgeVec =(edge.end - edge.start).normalize();
	Vec2 StoP = point - edge.start;
	//�n�_����̋��������߂�
	float dis = StoP.dot(edgeVec);
	return edge.start + (edgeVec * dis);
}


bool isCrossBbox(const BBox& b1, const BBox& b2) {
	//x���𒲂ׂ�
	if (b1.point.x + b1.width <= b2.point.x || b2.point.x + b2.width <= b1.point.x) {
		return false;
	}
	//y���𒲂ׂ�
	if (b1.point.y + b1.height <= b2.point.y || b2.point.y + b2.height <= b1.point.y) {
		return false;
	}
	return true;
}