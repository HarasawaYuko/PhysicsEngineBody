#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//�S����f��bias

//�v���g�^�C�v�錾
Matrix getRtilda(const Vec2&);

void Solver::initialize(const float timeStep) {
}

void Solver::solve(const std::vector<Object*>& objects ,std::vector<Collision>& cols) {
	//�\���o�[�{�f�B���쐬
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngle(), 1.f/obj->getM(), 1.f/obj->getI());
		}
		else {
			//�����Ȃ����̂� m,I = ���Ƃ��Čv�Z
			solverBodies[i] = SolverBody(obj->getAngle(), 0 ,0);
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
			vA = vA + objA->getV();
			vB = vB + objB->getV();
			//��]���x���v�Z
			vA = vA + objA->getCirV(cp.pointA);
			vB = vB + objB->getCirV(cp.pointB);
			//���Α��x
			Vec2 Vab = vA - vB;
/*���Α��x�擾�m�F*/

			//�s��K���v�Z
			Matrix K = Matrix(2, 2);
			float mSum = 1.f / objA->getM() + 1.f / objB->getM();
			Matrix mMat = Matrix(2 , 2);
			mMat.identity();
			K = K + (mMat * mSum);
			Matrix rAmat = getRtilda(cp.pointA) * (1.f/objA->getI());
			Matrix rBmat = getRtilda(cp.pointB) * (1.f/objB->getI());
			K = (K - rAmat) - rBmat;

			/*�S���̃Z�b�g�A�b�v*/
			//�S�������擾
			Vec2 axis = cp.normal;
			//�S�����̕���
			Matrix axis_ = Matrix(axis);
			Matrix denom_ = (K.product(axis_)).trans().product(axis_);
			assert(denom_.row == 1 && denom_.column == 1);//�f�o�b�O�p�@�P�s�ɏ���������
			float denom = denom_.matrix[0][0];
/*denom != 0*/
			//���������̍S�����Z�b�g
			cp.constraint[0].denomInv = 1.f / denom;
			cp.constraint[0].f = -(1.0f + restitution) * Vab.dot(axis);//���x�␳(f�̕���)
			cp.constraint[0].f -= (bias * min(0.0f , cp.depth))/(1.f/(float)FPS);//�ʒu�␳ �߂荞�݉����p
			cp.constraint[0].f *= cp.constraint[0].denomInv;//�����������
			//�S���͂̍ő�l�ƍŏ��l��ݒ�
			cp.constraint[0].lowerF = 0.0f;
			cp.constraint[0].upperF = FLT_MAX;
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
			Constraint& constraint = cp.constraint[0];
			float deltaImpulse = constraint.f;//���͂��擾
			Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA , bodyA.deltaRotaV);;//A�̑��x�ω���
			Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV);//B�̑��x�ω���
			//�S���͂��Z�o
			deltaImpulse -= constraint.denomInv * constraint.axis.dot(deltaVelocityA - deltaVelocityB);
/*deltaImpulse != 0 �m�F*/
			//�S���͂��N�����v
			clamp(deltaImpulse , constraint.lowerF , constraint.upperF);
			
			//�X�V���鑬�x���v�Z
			bodyA.deltaLinearV = bodyA.deltaLinearV +  (constraint.axis * (deltaImpulse * bodyA.massInv));
			bodyA.deltaRotaV += deltaImpulse * bodyA.inertiaInv;
			bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
			bodyB.deltaRotaV -= deltaImpulse * bodyB.inertiaInv;
			//���C�͂ɂ�鑬�x�ω��v�Z
		} 
	}
	//���x���X�V
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addV(solverBodies[i].deltaLinearV / (float)FPS);
		objects[i]->addVang(solverBodies[i].deltaRotaV /(float)FPS);
	}
	delete[] solverBodies;
}

void Solver::solve(World* world) {
	//�\���o�[�{�f�B���쐬
	std::vector<Object*> objects = world->objects;
	std::vector<Pair>& pairs = world->pairs;
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngle(), 1.f / obj->getM(), 1.f / obj->getI());
		}
		else {
			//�����Ȃ����̂� m,I = ���Ƃ��Čv�Z
			solverBodies[i] = SolverBody(obj->getAngle(), 0, 0);
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
			vA = vA + objA->getV();
			vB = vB + objB->getV();
			//��]���x���v�Z
			vA = vA + (objA->getCirV(cp.pointA ) );
			vB = vB + (objB->getCirV(cp.pointB ) );
			//���Α��x
			Vec2 Vab = vA - vB;
			//�s��K���v�Z
			Vec2 rA = cp.pointA.rotation(objA->getAngle());
			Vec2 rB = cp.pointB.rotation(objB->getAngle());
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
				Vec2 axis = cp.normal;
				//�S�����̕���
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix[0][0];//����
				//���������̍S�����Z�b�g
				cp.constraint[0].denomInv = 1.f / denom;
				cp.constraint[0].f = -(1.0f + restitution) * Vab.dot(axis);//���x�␳(f�̕���)
				cp.constraint[0].f -= (bias * min(0.0f, cp.depth + 0.1f )) / (1.f / (float)FPS);//�ʒu�␳ �߂荞�݉����p
				cp.constraint[0].f *= cp.constraint[0].denomInv;//�����������
				//�S���͂̍ő�l�ƍŏ��l��ݒ�
				cp.constraint[0].lowerF = 0.0f;
				cp.constraint[0].upperF = FLT_MAX;
				cp.constraint[0].axis = axis;
			}
			{
				//���C�͂��Z�b�g
				Vec2 axis = cp.normal.normal();
				//�S�����̕���
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix[0][0];
				cp.constraint[1].denomInv = 1.f / denom;
				cp.constraint[1].f = -Vab.dot(axis);       
				cp.constraint[1].f *= cp.constraint[1].denomInv;//�����������
				cp.constraint[1].axis = axis;
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
				Vec2 rA = cp.pointA.rotation(bodyA.angle);
				Vec2 rB = cp.pointB.rotation(bodyB.angle);
				//printfDx("rB %s\n" , rB.toString().c_str());
				{//��������
					Constraint& constraint = cp.constraint[0];
					float deltaImpulse = constraint.f;//���͂��擾
					Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA, bodyA.deltaRotaV).rotation(objA->getAngle());//A�̑��x�ω���
					Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV).rotation(objB->getAngle());//B�̑��x�ω���
					//�S���͂��Z�o
					deltaImpulse -= constraint.denomInv * constraint.axis.dot(deltaVelocityA - deltaVelocityB);
					float oldImpulse = constraint.accumImpulse;
					constraint.accumImpulse = clamp(oldImpulse + deltaImpulse , constraint.lowerF , constraint.upperF);
					deltaImpulse = constraint.accumImpulse - oldImpulse;
					//�X�V���鑬�x���v�Z
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis * deltaImpulse)) * bodyB.inertiaInv;
				}
				float maxFriction = col->getFri() * abs(cp.constraint[0].accumImpulse);
				cp.constraint[1].lowerF = -maxFriction;
				cp.constraint[1].upperF = maxFriction;
				{//���C�͂̍S�����Z
					//���C�͂ɂ�鑬�x�ω��v�Z
					Constraint constraint = cp.constraint[1];
					float deltaImpulse = constraint.f;//���͂��擾
					Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA, bodyA.deltaRotaV).rotation(objA->getAngle());//A�̑��x�ω���
					Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV).rotation(objB->getAngle());//B�̑��x�ω���
					//�S���͂��Z�o
					deltaImpulse -= constraint.denomInv * (constraint.axis.dot(deltaVelocityA - deltaVelocityB));
					float oldImpulse = constraint.accumImpulse;
					constraint.accumImpulse = clamp(oldImpulse + deltaImpulse , constraint.lowerF , constraint.upperF);
					deltaImpulse = constraint.accumImpulse - oldImpulse;
					//�X�V���鑬�x���v�Z
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis * deltaImpulse)) * bodyB.inertiaInv;
					//printfDx("��%s\n" , constraint.axis.toString().c_str());
				}
			}
		}
	}
	//���x���X�V
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addV(solverBodies[i].deltaLinearV);
		objects[i]->addVang(solverBodies[i].deltaRotaV);
	}
	delete[] solverBodies;
}

Matrix getRtilda(const Vec2& r) {
	Matrix mat = Matrix(2 ,2);
	mat.matrix[0][0] = r.y * r.y;
	mat.matrix[1][0] = -1 * r.y * r.x;
	mat.matrix[0][1] = -1 * r.y * r.x;
	mat.matrix[1][1] = r.x * r.x;
	return mat;
}