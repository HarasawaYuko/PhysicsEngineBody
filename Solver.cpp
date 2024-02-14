#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//拘束力fのbias

//プロトタイプ宣言
Matrix getRtilda(const Vec2&);

void Solver::solve(const std::vector<Object*>& objects ,std::vector<Collision>& cols) {
	//ソルバーボディを作成
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngleRad(), 1.f/obj->getMass(), 1.f/obj->getIne());
		}
		else {
			//動かない物体は m,I = ∞として計算
			solverBodies[i] = SolverBody(obj->getAngleRad(), 0 ,0);
		}
	}


	//拘束を設定
	for (int i = 0; i < cols.size(); i++) {
		//プロキシを作成
		Collision& col = cols[i];//衝突

		Object* objA = cols[i].getObj1();
		SolverBody bobyA = solverBodies[objA->getIndex()];
		Object* objB = cols[i].getObj2();
		SolverBody bodyB = solverBodies[objB->getIndex()];

		//摩擦係数と跳ね返り係数の設定
		col.setFri(sqrt(objA->getFri() * objB->getFri()));
	
		//跳ね返り係数は新規に検出された衝突のみに適用
		float restitution;
		if (true/*とりあえず反発係数はなし*/) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}

		for (int j = 0; j < col.getContactNum(); j++) {
			ContactPoint& cp = col.getCp(j);

			//相対速度(ワールド座標)を計算
			Vec2 vA = Vec2();//ワールド座標での衝突点に置ける速度
			Vec2 vB = Vec2();
			//並進速度を計算
			vA = vA + objA->getVelocity();
			vB = vB + objB->getVelocity();
			//回転速度を計算
			vA = vA + objA->rotationVelocityVec(cp.pointA_);
			vB = vB + objB->rotationVelocityVec(cp.pointB_);
			//相対速度
			Vec2 Vab = vA - vB;
/*相対速度取得確認*/

			//行列Kを計算
			Matrix K = Matrix(2, 2);
			float mSum = 1.f / objA->getMass() + 1.f / objB->getMass();
			Matrix mMat = Matrix(2 , 2);
			mMat.identity();
			K = K + (mMat * mSum);
			Matrix rAmat = getRtilda(cp.pointA_) * (1.f/objA->getIne());
			Matrix rBmat = getRtilda(cp.pointB_) * (1.f/objB->getIne());
			K = (K - rAmat) - rBmat;

			/*拘束のセットアップ*/
			//拘束軸を取得
			Vec2 axis = cp.normal_vec_;
			//拘束式の分母
			Matrix axis_ = Matrix(axis);
			Matrix denom_ = (K.product(axis_)).trans().product(axis_);
			assert(denom_.row_num_ == 1 && denom_.column_num_ == 1);//デバッグ用　１行に書き換える
			float denom = denom_.matrix_[0][0];
/*denom != 0*/
			//反発方向の拘束をセット
			cp.constraint_[0].f_denominator_ = 1.f / denom;
			cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//速度補正(fの分母)
			cp.constraint_[0].f_ -= (bias * min(0.0f , cp.depth_))/(1.f/(float)FPS);//位置補正 めり込み解消用
			cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//分母を加える
			//拘束力の最大値と最小値を設定
			cp.constraint_[0].lower_f_ = 0.0f;
			cp.constraint_[0].upper_f_ = FLT_MAX;
			//摩擦力をセット
			//一旦パス
		}
	}

	/*拘束の演算*/
	for (int i = 0; i < cols.size(); i++) {
		Collision& col = cols[i];
		Object* objA = cols[i].getObj1();
		SolverBody& bodyA = solverBodies[objA->getIndex()];
		Object* objB = cols[i].getObj2();
		SolverBody& bodyB = solverBodies[objB->getIndex()];
		for (int j = 0; j < col.getContactNum() ; j++) {
			ContactPoint& cp = col.getCp(j);
			Constraint& constraint = cp.constraint_[0];
			float deltaImpulse = constraint.f_;//撃力を取得
			Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_ , bodyA.deltaRotaV);;//Aの速度変化量
			Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV);//Bの速度変化量
			//拘束力を算出
			deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
/*deltaImpulse != 0 確認*/
			//拘束力をクランプ
			clamp(deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
			
			//更新する速度を計算
			bodyA.deltaLinearV = bodyA.deltaLinearV +  (constraint.axis_ * (deltaImpulse * bodyA.massInv));
			bodyA.deltaRotaV += deltaImpulse * bodyA.inertiaInv;
			bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
			bodyB.deltaRotaV -= deltaImpulse * bodyB.inertiaInv;
			//摩擦力による速度変化計算
		} 
	}
	//速度を更新
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addVel(solverBodies[i].deltaLinearV / (float)FPS);
		objects[i]->addVelAng(solverBodies[i].deltaRotaV /(float)FPS);
	}
	delete[] solverBodies;
}

void Solver::solve(World* world) {
	//ソルバーボディを作成
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
			//動かない物体は m,I = ∞として計算
			solverBodies[i] = SolverBody(obj->getAngleRad(), 0, 0);
		}
		solverBodies[i].deltaLinearV = Vec2();
		solverBodies[i].deltaRotaV = 0.f;
	}
/*solver body 確認*/
	//拘束を設定
	for (int i = 0; i < pairs.size(); i++) {
		//プロキシを作成
		Pair& pair = pairs[i];
		Collision* col = pair.getCol();//衝突

		Object* objA = pair.getObj0();
		SolverBody bodyA = solverBodies[objA->getIndex()];
		Object* objB = pair.getObj1();
		SolverBody bodyB = solverBodies[objB->getIndex()];

		//摩擦係数と跳ね返り係数の設定
		col->setFri(sqrt(objA->getFri() * objB->getFri()));
		//跳ね返り係数は新規に検出された衝突のみに適用
		float restitution;
		if (pair.getType() == Keep) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}
		for (int j = 0; j < col->getContactNum(); j++) {
			ContactPoint& cp = col->getCp(j);

			//相対速度(ワールド座標)を計算
			Vec2 vA = Vec2();//ワールド座標での衝突点に置ける速度
			Vec2 vB = Vec2();
			//並進速度を計算
			vA = vA + objA->getVelocity();
			vB = vB + objB->getVelocity();
			//回転速度を計算
			vA = vA + (objA->rotationVelocityVec(cp.pointA_ ) );
			vB = vB + (objB->rotationVelocityVec(cp.pointB_ ) );
			//相対速度
			Vec2 Vab = vA - vB;
			//行列Kを計算
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

/******拘束のセットアップ******/
			{
				//拘束軸を取得
				Vec2 axis = cp.normal_vec_;
				//拘束式の分母
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];//分母
				//反発方向の拘束をセット
				cp.constraint_[0].f_denominator_ = 1.f / denom;
				cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//速度補正(fの分母)
				cp.constraint_[0].f_ -= (bias * min(0.0f, cp.depth_ + 0.1f )) / (1.f / (float)FPS);//位置補正 めり込み解消用
				cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//分母を加える
				//拘束力の最大値と最小値を設定
				cp.constraint_[0].lower_f_ = 0.0f;
				cp.constraint_[0].upper_f_ = FLT_MAX;
				cp.constraint_[0].axis_ = axis;
			}
			{
				//摩擦力をセット
				Vec2 axis = cp.normal_vec_.normalCCW();
				//拘束式の分母
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];
				cp.constraint_[1].f_denominator_ = 1.f / denom;
				cp.constraint_[1].f_ = -Vab.dot(axis);       
				cp.constraint_[1].f_ *= cp.constraint_[1].f_denominator_;//分母を加える
				cp.constraint_[1].axis_ = axis;
			}
		}
	}

/********拘束の演算*******/
	//printfDx("-------------演算---------------\n");
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
				{//反発方向
					Constraint& constraint = cp.constraint_[0];
					float deltaImpulse = constraint.f_;//撃力を取得
					Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_, bodyA.deltaRotaV).rotationCCW(objA->getAngleRad());//Aの速度変化量
					Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV).rotationCCW(objB->getAngleRad());//Bの速度変化量
					//拘束力を算出
					deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
					float oldImpulse = constraint.accum_impulse_;
					constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
					deltaImpulse = constraint.accum_impulse_ - oldImpulse;
					//更新する速度を計算
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis_ * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertiaInv;
				}
				float maxFriction = col->getFri() * abs(cp.constraint_[0].accum_impulse_);
				cp.constraint_[1].lower_f_ = -maxFriction;
				cp.constraint_[1].upper_f_ = maxFriction;
				{//摩擦力の拘束演算
					//摩擦力による速度変化計算
					Constraint constraint = cp.constraint_[1];
					float deltaImpulse = constraint.f_;//撃力を取得
					Vec2 deltaVelocityA = bodyA.deltaLinearV + rotationVelocity(cp.pointA_, bodyA.deltaRotaV).rotationCCW(objA->getAngleRad());//Aの速度変化量
					Vec2 deltaVelocityB = bodyB.deltaLinearV + rotationVelocity(cp.pointB_, bodyB.deltaRotaV).rotationCCW(objB->getAngleRad());//Bの速度変化量
					//拘束力を算出
					deltaImpulse -= constraint.f_denominator_ * (constraint.axis_.dot(deltaVelocityA - deltaVelocityB));
					float oldImpulse = constraint.accum_impulse_;
					constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse , constraint.lower_f_ , constraint.upper_f_);
					deltaImpulse = constraint.accum_impulse_ - oldImpulse;
					//更新する速度を計算
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis_ * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis_ * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertiaInv;
					//printfDx("軸%s\n" , constraint.axis.toString().c_str());
				}
			}
		}
	}
	//速度を更新
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