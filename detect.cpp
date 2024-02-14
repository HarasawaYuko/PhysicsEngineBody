#include "detect.h"
#include "Circle.h"
#include "Convex.h"
#include "DEBUG.h"

//�v���g�^�C�v�錾
void projection(const Vec2& ,const Convex* , float*, float*);
void getDepth(float* d1, float* d2, const Vec2& axis, const Convex* con1, const Convex* con2);
Vec2 calContactPointOnEdge(const Vec2& , const Segment&);
bool isCrossBbox(const BBox&, const BBox&);

//�Փ˓_�̎擾
void computeContactCoord(Vec2*, Vec2* ,Convex* , Convex* , const float , const Vec2& );
void computeContactCoord(Vec2*, Vec2* ,const Circle* , const Convex* , const Vec2& , const float , const Vec2& );
void computeContactCoord(Vec2*, Vec2* ,Circle* , Circle* , const float , const Vec2& );
//����������
bool separationJudgeIsCross(const Vec2& , const Convex* , const Convex* , float& , Vec2& );

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
//�ʕ�Ɠʕ�̏Փ˔���̃I�[�o�[���[�h
bool Detect::narrow(Convex* convex1 , Convex* convex2 , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//����������
	float depth_max = -INF;
	Vec2 axis_max_depth;//������

	const std::vector<Vec2> points_world_con1 = convex1->getPointsWorld();
	const std::vector<Vec2> points_world_con2 = convex2->getPointsWorld();
	
	//���_�����_�̃x�N�g���̕���������
	for (const auto& point1 : points_world_con1 ) {
		for (const auto& point2 : points_world_con2 ) {
			const Vec2 axis = (point1 - point2).normalize();
			//������������s��
			if (!separationJudgeIsCross(axis , convex1 , convex2 , depth_max , axis_max_depth )) {
				return false;
			}
		}
	}
	//�ӂ̖@���x�N�g���̕���������
	//convex1
	for (int i = 0; i < convex1->getPointNum(); i++) {
		//��(�ӂ̖@���x�N�g��)���擾
		const Segment edge = convex1->getEdgeW(i);
		const Vec2 axis = (edge.start_ - edge.end_).normalize().normalCCW();
		
		//������������s��
		if (!separationJudgeIsCross(axis, convex1, convex2, depth_max, axis_max_depth)) {
			return false;
		}
	}
	//convex2
	for (int i = 0; i < convex2->getPointNum(); i++) {
		//��(�ӂ̖@���x�N�g��)���擾
		const Segment edge = convex2->getEdgeW(i);
		const Vec2 axis = (edge.start_ - edge.end_).normalize().normalCCW();

		//������������s��
		if (!separationJudgeIsCross(axis, convex1, convex2, depth_max, axis_max_depth)) {
			return false;
		}
	}

	//�ђʐ[�x���̐ݒ�
	*depth = depth_max;
	*n_vec = axis_max_depth;

	//�Փ˂��Ă���ꍇ�͏Փ˓_���擾����
	computeContactCoord( coord, coord_edge , convex1 , convex2 , depth_max , axis_max_depth );
}

//�~�Ɠʕ�̏Փ˔���̃I�[�o�[���[�h
bool Detect::narrow(Circle* circle  , Convex* convex  , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//�ŒZ���������a���Z����ΏՓ˂Ƃ���
	const Vec2 center_cir = circle->getCenter();
	const float r = circle->getR();

	Vec2 point_min;
	float distance_min = FLT_MAX;

	//�ʕ�̒��_���~�܂ł̋���
	for (int i = 0; i < convex->getPointNum(); i++) {
		const Vec2 point = convex->getPointW(i);
		const float distance = point.distance(center_cir);
		if (distance < distance_min) {
			distance_min = distance;
			point_min = point;
		}
	}
	//�ʕ�̕Ӂ��~�܂ł̋���
	bool edge_has_minpoint = false;
	Segment edge_min;
	for (int i = 0; i < convex->getPointNum(); i++) {
		const Segment edge = convex->getEdgeW(i);
		const float distance= distanceSegPoint(center_cir , edge);
		if (distance < distance_min) {
			distance_min = distance;
			edge_min = edge;
			edge_has_minpoint = true;
		}
	}

	//�ŒZ�����ŏՓ˔���
	if (distance_min > r) {
		return false;
	}

	//�ŋߓ_���Z�o����
	if ( edge_has_minpoint ) {
		point_min = calContactPointOnEdge(center_cir , edge_min );
	}

	//�ђʐ[�x�Ɩ@���x�N�g�����擾
	*depth = distance_min - r;
	*n_vec = (center_cir - point_min).normalize();
	
	//�Փ˓_���擾
	computeContactCoord(coord , coord_edge , circle , convex , point_min , *depth , *n_vec);

	return true;
}

//�~�Ɖ~�̏Փ˔���̃I�[�o�[���[�h
bool Detect::narrow(Circle* circle1 , Circle* circle2 , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//���S�Ԃ̋������擾
	const float distance_center = circle1->getCenter().distance(circle2->getCenter());
	const float r1 = circle1->getR();
	const float r2 = circle2->getR();
	if ((r1 + r2) < distance_center) {
		return false;
	}
	//�ђʐ[�x���擾
	*depth = distance_center - (r1 + r2);
	//�@���x�N�g��
	*n_vec = (circle1->getCenter() - circle2->getCenter()).normalize();

	computeContactCoord(coord, coord_edge, circle1, circle2, *depth, *n_vec);
	return true;
}

//�Փˍ��W���擾
void computeContactCoord( Vec2* coord, Vec2* coord_edge ,Convex* convex1 , Convex* convex2 , const float depth , const Vec2& n_vec ) {
	float distance_min = INF;
	//���_���ڐG�_�ƂȂ镨�̂̃C���f�b�N�X
	//A�̕��̂̒��_�Ȃ�0 ,B�̕��̂̒��_�Ȃ�1
	int point_obj_index = 0;
	int point_min_index;
	int edge_min_index;


	//����1���ђʐ[�x���኱���炷
	Vec2 move_vec = n_vec * (abs(depth) * 2.f);//���炷�x�N�g��
	convex1->move(move_vec);

	Convex* con_point = convex1;
	Convex* con_edge = convex2;
	for (int i = 0; i < 2; i++) {
		if (i == 1) {
			con_point = convex2;
			con_edge = convex1;
		}
		//�ŒZ���������߂�
		for (int it_p = 0; it_p < con_point->getPointNum(); it_p++) {
			for (int it_e = 0; it_e < con_edge->getPointNum(); it_e++) {
				const Vec2 point = con_point->getPointW(it_p);
				const Segment edge = con_edge->getEdgeW(it_e);
				const float dis = distanceSegPoint(point, edge);
				if (dis < distance_min) {
					//�L�^
					point_obj_index = i;
					distance_min = dis;
					point_min_index = it_p;
					edge_min_index = it_e;
					
				}
			}
		}
	}
	printfDx("point_obj %d \n" , point_obj_index);
	//���炵�����߂�
	convex1->move(move_vec * -1);

	if (point_obj_index == 0) {
		con_point = convex1;
		con_edge = convex2;
	}
	else {
		con_point = convex2;
		con_edge = convex1;
	}

	Vec2 min_point = con_point->getPointW(point_min_index);
	Segment min_edge = con_edge->getEdgeW(edge_min_index);

	coord[0] = WtoL(min_point , convex1->getCenter(), convex1->getAngleRad());
	coord[1] = WtoL(min_point , convex2->getCenter(), convex2->getAngleRad());
	coord_edge[point_obj_index] = coord[point_obj_index];
	const int edge_obj_index = (point_obj_index + 1) % 2;
	coord_edge[edge_obj_index] = WtoL(calContactPointOnEdge(min_point, min_edge), con_edge->getCenter(), con_edge->getAngleRad());
}

void computeContactCoord(Vec2* coord , Vec2* coord_edge , const Circle* circle  ,const Convex* convex , const Vec2& point_min , const float depth, const Vec2& vec_con_to_cir ) {
	//coord[0]�͉~��Acoord[1]�͓ʕ�
	coord[0] = WtoL(point_min , circle->getCenter() , circle->getAngleRad());
	coord[1] = WtoL(point_min , convex->getCenter() , convex->getAngleRad());
	coord_edge[1] = coord[1];
	//�~����̏Փ˓_���v�Z����
	//�~�̒��S����~����̓_
	Vec2 contactpoint_circle = circle->getCenter() - (vec_con_to_cir * circle->getR());
	coord_edge[0] = WtoL(contactpoint_circle , circle->getCenter() , circle->getAngleRad());
}

void computeContactCoord(Vec2* coord, Vec2* coord_edge ,Circle* circle1 , Circle* circle2 , const float depth , const Vec2& vec_2to1) {
	//�Փ˓_���擾 2�̉~����
	const Vec2 center1 = circle1->getCenter();
	const Vec2 center2 = circle2->getCenter();
	Vec2 cir2_edge_contactpoint = center2 + (vec_2to1.normalize() * circle2->getR());
	coord[0] = WtoL(cir2_edge_contactpoint, center1 , circle1->getAngleRad());
	coord[1] = WtoL(cir2_edge_contactpoint, center2 , circle2->getAngleRad());
	coord_edge[0] = WtoL( center1 - ((vec_2to1 ) * circle1->getR()), center1 , circle1->getAngleRad());
	coord_edge[1] = coord[1];
}

/**************************************************************/

//������������s��
//�Փ˂��Ă��邩��return���A
//depth_max�ƁAaxis_max��ݒ肷��
bool separationJudgeIsCross(const Vec2& axis , const Convex* con1 , const Convex* con2 , float& depth_max , Vec2& axis_max ) {
	float depth1;
	float depth2;
	
	getDepth(&depth1, &depth2, axis, con1, con2);
	if (depth1 > 0.f || depth2 > 0.f) {
		return false;
	}

	if ( depth1 > depth_max ) {
		depth_max = depth1;
		axis_max = axis;
	}
	if ( depth2 > depth_max ) {
		depth_max = depth2;
		axis_max = axis * -1;
	}
	return true;
}

//axis��box(convex)�𓊉e���čő�ƍŏ���Ԃ�
void projection(const Vec2& axis, const Convex* con, float* min_result, float* max_result) {
	float min = INF;
	float max = -INF;
	//�S�Ă̒��_�𓊉e
	for (int i = 0; i < con->getPointNum(); i++) {
		float dot = axis.dot(con->getPointW(i));
		min = min( min , dot);
		max = max( max , dot);
	}
	*min_result = min;
	*max_result = max;
}

void getDepth( float* d1 , float* d2 , const Vec2& axis , const Convex* con1 , const Convex* con2 ) {
	//���e���W�̎擾
	float max1, min1;
	float max2, min2;
	projection( axis , con1 , &min1 , &max1 );
	projection( axis , con2 , &min2 , &max2 );

	//�ђʐ[�x���擾
	*d1 = min1 - max2;
	*d2 = min2 - max1;
}

//�����e�̌��ʁA�����������邩���肷��@�������Ă��Ȃ�(d>0)�̎���true
void getDepth(const float min1 , const float max1 ,const float min2 , const float max2  , float* d1 , float* d2) {

	//�ђʐ[�x�����߂āA�����Ŕ��肷��
	*d1 = min1 - max2;
	*d2 = min2 - max1;
}

Vec2 calContactPointOnEdge(const Vec2& point , const Segment& edge) {
	//�Ӄx�N�g���̐��K��
	Vec2 edge_vec =(edge.end_ - edge.start_);
	Vec2 start_point_vec = point - edge.start_;
	//�n�_����̋��������߂�
	float dis = start_point_vec.dot(edge_vec.normalize());
	dis = clamp(dis , 0.f , edge_vec.norm());
	return edge.start_ + (edge_vec.normalize() * dis);
}


bool isCrossBbox(const BBox& b1, const BBox& b2) {
	//x���𒲂ׂ�
	if (b1.origin_.x_ + b1.width_ <= b2.origin_.x_ || b1.origin_.x_ >= b2.origin_.x_ + b2.width_ ) {
		return false;
	}
	//y���𒲂ׂ�
	if (b1.origin_.y_ + b1.height_ <= b2.origin_.y_ ||b1.origin_.y_ >= b2.origin_.y_ + b2.height_ ) {
		return false;
	}
	return true;
}