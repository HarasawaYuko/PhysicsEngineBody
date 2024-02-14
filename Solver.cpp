#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//�S����f��bias

//�v���g�^�C�v�錾
Matrix getRtilda(const Vec2&);

void Solver::solve(const std::vector<Object*>& objects ,std::vector<Collision>& cols) {
	//�\���o�[�{�f�B���쐬
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngleRad(), 1.f/obj->getMass(), 1.f/obj->getIne());
		}
		else {
			//�����Ȃ����̂� m,I = ���Ƃ��Čv�Z
			solverBodies[i] = SolverBody(obj->getAngleRad(), 0 ,0);
		}
	}


	//�S����ݒ�
	for (int i = 0; i < cols.size(); i++) {
		//�v���L�V���쐬
		Collision& col = cols[i];//�Փ�

		Object* objA = cols[i].getObj1();
		SolverBody bobyA = solverBodies[objA->getIndex()];
		Object* objB = cols[i].getObj2();
		SolverBody bodyB = solverBodies[objB->getIndex()];

		//���C�W���ƒ��˕Ԃ�W���̐ݒ�
		col.setFri(sqrt(objA->getFri() * objB->getFri()));
	
		//���˕Ԃ�W���͐V�K�Ɍ��o���ꂽ�Փ˂݂̂ɓK�p
		float restitution;
		if (true/*�Ƃ肠���������W���͂Ȃ�*/) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}

		for (int j = 0; j < col.getContactNum(); j++) {
			ContactPoint& cp = col.getCp(j);

			//���Α��x(���[���h���W)���v�Z
			Vec2 vA = Vec2();//���[���h���W�ł̏Փ˓_�ɒu���鑬�x
			Vec2 vB = Vec2();
			//���i���x���v�Z
			vA = vA + objA->getVelocity();
			vB = vB + objB->getVelocity();
			//��]���x���v�Z
			vA = vA + objA->rotationVelocityVec(cp.pointA_);
			vB = vB + objB->rotationVelocityVec(cp.pointB_);
			//���Α��x
			Vec2 Vab = vA - vB;
/*���Α��x�擾�m�F*/

			//�s��K���v�Z
			Matrix K = Matrix(2, 2);
			float mSum = 1.f / objA->getMass() + 1.f / objB->getMass();
			Matrix mMat = Matrix(2 , 2);
			mMat.identity();
			K = K + (mMat * mSum);
			Matrix rAmat = getRtilda(cp.pointA_) * (1.f/objA->getIne());
			Matrix rBmat = getRtilda(cp.pointB_) * (1.f/objB->getIne());
			K = (K - rAmat) - rBmat;

			/*�S���̃Z�b�g�A�b�v*/
			//�S�������擾
			Vec2 axis = cp.normal_vec_;
			//�S�����̕���
			Matrix axis_ = Matrix(axis);
			Matrix denom_ = (K.product(axis_)).trans().product(axis_);
			assert(denom_.row_num_ == 1 && denom_.column_num_ == 1);//�f�o�b�O�p�@�P�s�ɏ���������
			float denom = denom_.matrix_[0][0];
/*denom != 0*/
			//���������̍S�����Z�b�g
			cp.constraint_[0].f_denominator_ = 1.f / denom;
			cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//���x�␳(f�̕���)
			cp.constraint_[0].f_ -= (bias * min(0.0f , cp.depth_))/(1.f/(float)FPS);//�ʒu�␳ �߂荞�݉����p
			cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//�����������
			//�S���͂̍ő�l�ƍŏ��l��ݒ�
			cp.constraint_[0].lower_f_ = 0.0f;
			cp.constraint_[0].upper_f_ = FLT_MAX;
			//���C�͂��Z�b�g
			//��U�p�X
		}
	}

	/*�S���̉��Z*/
	for (int i = 0; i < cols.size(); i++) {
		Collision& col = cols[i];
		Object* objA = cols[i].getObj1();
		SolverBody& bodyA = solverBodies[objA->getIndex()];
		Object* objB = cols[i].getObj2();
		SolverBody& bodyB = solverBodies[objB->getIndex()];
		for (int j = 0; j < col.getContactNum() ; j++) {
			ContactPoint& cp = col.getCp(j);
			Constraint& constraint = cp.constraint_[0];
			float deltaImpulse = constraint.f_;//���͂��擾
			Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_ , bodyA.deltaRotaV);;//A�̑��x�ω���
			Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV);//B�̑��x�ω���
			//�S���͂��Z�o
			deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
/*deltaImpulse != 0 �m�F*/
			//�S���͂��N�����v
			clamp(deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
			
			//�X�V���鑬�x���v�Z
			bodyA.deltaLinearV = bodyA.deltaLinearV +  (constraint.axis_ * (deltaImpulse * bodyA.massInv));
			bodyA.deltaRotaV += deltaImpulse * bodyA.inertiaInv;
			bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
			bodyB.deltaRotaV -= deltaImpulse * bodyB.inertiaInv;
			//���C�͂ɂ�鑬�x�ω��v�Z
		} 
	}
	//���x���X�V
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addVel(solverBodies[i].deltaLinearV / (float)FPS);
		objects[i]->addVelAng(solverBodies[i].deltaRotaV /(float)FPS);
	}
	delete[] solverBodies;
}

void Solver::solve(World* world) {
	//�\���o�[�{�f�B���쐬
	std::vector<Object*> objects = world->objects_;
	std::vector<Pair>& pairs = world->pairs_;
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngleRad(), 1.f / obj->getMass(), 1.f / obj->getIne());
		}
		else {
			//�����Ȃ����̂� m,I = ���Ƃ��Čv�Z
			solverBodies[i] = SolverBody(obj->getAngleRad(), 0, 0);
		}
		solverBodies[i].deltaLinearV = Vec2();
		solverBodies[i].deltaRotaV = 0.f;
	}
/*solver body �m�F*/
	//�S����ݒ�
	for (int i = 0; i < pairs.size(); i++) {
		//�v���L�V���쐬
		Pair& pair = pairs[i];
		Collision* col = pair.getCol();//�Փ�

		Object* objA = pair.getObj0();
		SolverBody bodyA = solverBodies[objA->getIndex()];
		Object* objB = pair.getObj1();
		SolverBody bodyB = solverBodies[objB->getIndex()];

		//���C�W���ƒ��˕Ԃ�W���̐ݒ�
		col->setFri(sqrt(objA->getFri() * objB->getFri()));
		//���˕Ԃ�W���͐V�K�Ɍ��o���ꂽ�Փ˂݂̂ɓK�p
		float restitution;
		if (pair.getType() == Keep) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}
		for (int j = 0; j < col->getContactNum(); j++) {
			ContactPoint& cp = col->getCp(j);

			//���Α��x(���[���h���W)���v�Z
			Vec2 vA = Vec2();//���[���h���W�ł̏Փ˓_�ɒu���鑬�x
			Vec2 vB = Vec2();
			//���i���x���v�Z
			vA = vA + objA->getVelocity();
			vB = vB + objB->getVelocity();
			//��]���x���v�Z
			vA = vA + (objA->rotationVelocityVec(cp.pointA_ ) );
			vB = vB + (objB->rotationVelocityVec(cp.pointB_ ) );
			//���Α��x
			Vec2 Vab = vA - vB;
			//�s��K���v�Z
			Vec2 rA = cp.pointA_.rotationCCW(objA->getAngleRad());
			Vec2 rB = cp.pointB_.rotationCCW(objB->getAngleRad());
			Matrix K = Matrix(2, 2);
			float mSum = bodyA.massInv + bodyB.massInv;
			Matrix mMat = Matrix(2, 2);
			mMat.identity();
			K = K + (mMat * mSum);
			Matrix rAmat = getRtilda(rA) * (bodyA.inertiaInv);
			Matrix rBmat = getRtilda(rB) * (bodyB.inertiaInv);
			K = (K + rAmat) + rBmat;

/******�S���̃Z�b�g�A�b�v******/
			{
				//�S�������擾
				Vec2 axis = cp.normal_vec_;
				//�S�����̕���
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];//����
				//���������̍S�����Z�b�g
				cp.constraint_[0].f_denominator_ = 1.f / denom;
				cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//���x�␳(f�̕���)
				cp.constraint_[0].f_ -= (bias * min(0.0f, cp.depth_ + 0.1f )) / (1.f / (float)FPS);//�ʒu�␳ �߂荞�݉����p
				cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//�����������
				//�S���͂̍ő�l�ƍŏ��l��ݒ�
				cp.constraint_[0].lower_f_ = 0.0f;
				cp.constraint_[0].upper_f_ = FLT_MAX;
				cp.constraint_[0].axis_ = axis;
			}
			{
				//���C�͂��Z�b�g
				Vec2 axis = cp.normal_vec_.normalCCW();
				//�S�����̕���
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];
				cp.constraint_[1].f_denominator_ = 1.f / denom;
				cp.constraint_[1].f_ = -Vab.dot(axis);       
				cp.constraint_[1].f_ *= cp.constraint_[1].f_denominator_;//�����������
				cp.constraint_[1].axis_ = axis;
			}
		}
	}

/********�S���̉��Z*******/
	//printfDx("-------------���Z---------------\n");
	for (int k = 0; k < 10; k++) {
		for (int i = 0; i < pairs.size(); i++) {
			Collision* col = pairs[i].getCol();
			Object* objA = pairs[i].getObj0();
			SolverBody& bodyA = solverBodies[objA->getIndex()];
			Object* objB = pairs[i].getObj1();
			SolverBody& bodyB = solverBodies[objB->getIndex()];
			for (int j = 0; j < col->getContactNum(); j++) {
				ContactPoint& cp = col->getCp(j);
				Vec2 rA = cp.pointA_.rotationCCW(bodyA.angle);
				Vec2 rB = cp.pointB_.rotationCCW(bodyB.angle);
				//printfDx("rB %s\n" , rB.toString().c_str());
				{//��������
					Constraint& constraint = cp.constraint_[0];
					float deltaImpulse = constraint.f_;//���͂��擾
					Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_, bodyA.deltaRotaV).rotationCCW(objA->getAngleRad());//A�̑��x�ω���
					Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV).rotationCCW(objB->getAngleRad());//B�̑��x�ω���
					//�S���͂��Z�o
					deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
					float oldImpulse = constraint.accum_impulse_;
					constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
					deltaImpulse = constraint.accum_impulse_ - oldImpulse;
					//�X�V���鑬�x���v�Z
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis_ * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertiaInv;
				}
				float maxFriction = col->getFri() * abs(cp.constraint_[0].accum_impulse_);
				cp.constraint_[1].lower_f_ = -maxFriction;
				cp.constraint_[1].upper_f_ = maxFriction;
				{//���C�͂̍S�����Z
					//���C�͂ɂ�鑬�x�ω��v�Z
					Constraint constraint = cp.constraint_[1];
					float deltaImpulse = constraint.f_;//���͂��擾
					Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_, bodyA.deltaRotaV).rotationCCW(objA->getAngleRad());//A�̑��x�ω���
					Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV).rotationCCW(objB->getAngleRad());//B�̑��x�ω���
					//�S���͂��Z�o
					deltaImpulse -= constraint.f_denominator_ * (constraint.axis_.dot(deltaVelocityA - deltaVelocityB));
					float oldImpulse = constraint.accum_impulse_;
					constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
					deltaImpulse = constraint.accum_impulse_ - oldImpulse;
					//�X�V���鑬�x���v�Z
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis_ * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertiaInv;
					//printfDx("��%s\n" , constraint.axis.toString().c_str());
				}
			}
		}
	}
	//���x���X�V
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addVel(solverBodies[i].deltaLinearV);
		objects[i]->addVelAng(solverBodies[i].deltaRotaV);
	}
	delete[] solverBodies;
}

Matrix getRtilda(const Vec2& r) {
	Matrix mat = Matrix(2 ,2);
	mat.matrix_[0][0] = r.y_ * r.y_;
	mat.matrix_[1][0] = -1 * r.y_ * r.x_;
	mat.matrix_[0][1] = -1 * r.y_ * r.x_;
	mat.matrix_[1][1] = r.x_ * r.x_;
	return mat;
}