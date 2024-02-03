#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//拘束力fのbias

//プロトタイプ宣言
Matrix getRtilda(const Vec2&);

void Solver::initialize(const float timeStep) {
}

void Solver::solve(const std::vector<Object*>& objects ,std::vector<Collision>& cols) {
	//ソルバーボディを作成
	SolverBody* solverBodies;
	solverBodies = new SolverBody[objects.size()];
	for (int i = 0; i < objects.size(); i++) {
		Object* obj = objects[i];
		if (obj->isActive()) {
			solverBodies[i] = SolverBody(obj->getAngle(), 1.f/obj->getM(), 1.f/obj->getI());
		}
		else {
			//動かない物体は m,I = ∞として計算
			solverBodies[i] = SolverBody(obj->getAngle(), 0 ,0);
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
			vA = vA + objA->getV();
			vB = vB + objB->getV();
			//回転速度を計算
			vA = vA + objA->getCirV(cp.pointA);
			vB = vB + objB->getCirV(cp.pointB);
			//相対速度
			Vec2 Vab = vA - vB;
/*相対速度取得確認*/

			//行列Kを計算
			Matrix K = Matrix(2, 2);
			float mSum = 1.f / objA->getM() + 1.f / objB->getM();
			Matrix mMat = Matrix(2 , 2);
			mMat.identity();
			K = K + (mMat * mSum);
			Matrix rAmat = getRtilda(cp.pointA) * (1.f/objA->getI());
			Matrix rBmat = getRtilda(cp.pointB) * (1.f/objB->getI());
			K = (K - rAmat) - rBmat;

			/*拘束のセットアップ*/
			//拘束軸を取得
			Vec2 axis = cp.normal;
			//拘束式の分母
			Matrix axis_ = Matrix(axis);
			Matrix denom_ = (K.product(axis_)).trans().product(axis_);
			assert(denom_.row == 1 && denom_.column == 1);//デバッグ用　１行に書き換える
			float denom = denom_.matrix[0][0];
/*denom != 0*/
			//反発方向の拘束をセット
			cp.constraint[0].denomInv = 1.f / denom;
			cp.constraint[0].f = -(1.0f + restitution) * Vab.dot(axis);//速度補正(fの分母)
			cp.constraint[0].f -= (bias * min(0.0f , cp.depth))/(1.f/(float)FPS);//位置補正 めり込み解消用
			cp.constraint[0].f *= cp.constraint[0].denomInv;//分母を加える
			//拘束力の最大値と最小値を設定
			cp.constraint[0].lowerF = 0.0f;
			cp.constraint[0].upperF = FLT_MAX;
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
			Constraint& constraint = cp.constraint[0];
			float deltaImpulse = constraint.f;//撃力を取得
			Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA , bodyA.deltaRotaV);;//Aの速度変化量
			Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV);//Bの速度変化量
			//拘束力を算出
			deltaImpulse -= constraint.denomInv * constraint.axis.dot(deltaVelocityA - deltaVelocityB);
/*deltaImpulse != 0 確認*/
			//拘束力をクランプ
			clamp(deltaImpulse , constraint.lowerF , constraint.upperF);
			
			//更新する速度を計算
			bodyA.deltaLinearV = bodyA.deltaLinearV +  (constraint.axis * (deltaImpulse * bodyA.massInv));
			bodyA.deltaRotaV += deltaImpulse * bodyA.inertiaInv;
			bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
			bodyB.deltaRotaV -= deltaImpulse * bodyB.inertiaInv;
			//摩擦力による速度変化計算
		} 
	}
	//速度を更新
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addV(solverBodies[i].deltaLinearV / (float)FPS);
		objects[i]->addVang(solverBodies[i].deltaRotaV /(float)FPS);
	}
	delete[] solverBodies;
}

void Solver::solve(World* world) {
	//ソルバーボディを作成
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
			//動かない物体は m,I = ∞として計算
			solverBodies[i] = SolverBody(obj->getAngle(), 0, 0);
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
			vA = vA + objA->getV();
			vB = vB + objB->getV();
			//回転速度を計算
			vA = vA + (objA->getCirV(cp.pointA ) );
			vB = vB + (objB->getCirV(cp.pointB ) );
			//相対速度
			Vec2 Vab = vA - vB;
			//行列Kを計算
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

/******拘束のセットアップ******/
			{
				//拘束軸を取得
				Vec2 axis = cp.normal;
				//拘束式の分母
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix[0][0];//分母
				//反発方向の拘束をセット
				cp.constraint[0].denomInv = 1.f / denom;
				cp.constraint[0].f = -(1.0f + restitution) * Vab.dot(axis);//速度補正(fの分母)
				cp.constraint[0].f -= (bias * min(0.0f, cp.depth + 0.1f )) / (1.f / (float)FPS);//位置補正 めり込み解消用
				cp.constraint[0].f *= cp.constraint[0].denomInv;//分母を加える
				//拘束力の最大値と最小値を設定
				cp.constraint[0].lowerF = 0.0f;
				cp.constraint[0].upperF = FLT_MAX;
				cp.constraint[0].axis = axis;
			}
			{
				//摩擦力をセット
				Vec2 axis = cp.normal.normal();
				//拘束式の分母
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix[0][0];
				cp.constraint[1].denomInv = 1.f / denom;
				cp.constraint[1].f = -Vab.dot(axis);       
				cp.constraint[1].f *= cp.constraint[1].denomInv;//分母を加える
				cp.constraint[1].axis = axis;
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
				Vec2 rA = cp.pointA.rotation(bodyA.angle);
				Vec2 rB = cp.pointB.rotation(bodyB.angle);
				//printfDx("rB %s\n" , rB.toString().c_str());
				{//反発方向
					Constraint& constraint = cp.constraint[0];
					float deltaImpulse = constraint.f;//撃力を取得
					Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA, bodyA.deltaRotaV).rotation(objA->getAngle());//Aの速度変化量
					Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV).rotation(objB->getAngle());//Bの速度変化量
					//拘束力を算出
					deltaImpulse -= constraint.denomInv * constraint.axis.dot(deltaVelocityA - deltaVelocityB);
					float oldImpulse = constraint.accumImpulse;
					constraint.accumImpulse = clamp(oldImpulse + deltaImpulse , constraint.lowerF , constraint.upperF);
					deltaImpulse = constraint.accumImpulse - oldImpulse;
					//更新する速度を計算
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis * deltaImpulse)) * bodyB.inertiaInv;
				}
				float maxFriction = col->getFri() * abs(cp.constraint[0].accumImpulse);
				cp.constraint[1].lowerF = -maxFriction;
				cp.constraint[1].upperF = maxFriction;
				{//摩擦力の拘束演算
					//摩擦力による速度変化計算
					Constraint constraint = cp.constraint[1];
					float deltaImpulse = constraint.f;//撃力を取得
					Vec2 deltaVelocityA = bodyA.deltaLinearV + getVang(cp.pointA, bodyA.deltaRotaV).rotation(objA->getAngle());//Aの速度変化量
					Vec2 deltaVelocityB = bodyB.deltaLinearV + getVang(cp.pointB, bodyB.deltaRotaV).rotation(objB->getAngle());//Bの速度変化量
					//拘束力を算出
					deltaImpulse -= constraint.denomInv * (constraint.axis.dot(deltaVelocityA - deltaVelocityB));
					float oldImpulse = constraint.accumImpulse;
					constraint.accumImpulse = clamp(oldImpulse + deltaImpulse , constraint.lowerF , constraint.upperF);
					deltaImpulse = constraint.accumImpulse - oldImpulse;
					//更新する速度を計算
					bodyA.deltaLinearV = bodyA.deltaLinearV + (constraint.axis * (deltaImpulse * bodyA.massInv));
					bodyA.deltaRotaV += (rA.cross(constraint.axis * deltaImpulse)) * bodyA.inertiaInv;
					bodyB.deltaLinearV = bodyB.deltaLinearV - (constraint.axis * (deltaImpulse * bodyB.massInv));
					bodyB.deltaRotaV -= (rB.cross(constraint.axis * deltaImpulse)) * bodyB.inertiaInv;
					//printfDx("軸%s\n" , constraint.axis.toString().c_str());
				}
			}
		}
	}
	//速度を更新
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