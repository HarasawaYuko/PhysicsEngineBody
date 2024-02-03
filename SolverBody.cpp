#include "SolvetBody.h"
#include "UIMaterial.h"

SolverBody::SolverBody(const float ang , const float mInv , const float iInv) 
	:deltaLinearV(Vec2(0.f , 0.f)) , deltaRotaV(0.f) , angle(ang) , massInv(mInv) , inertiaInv(iInv)
{}

std::string SolverBody::toString()const {
	std::string str;
	str += "épê®:" + std::to_string(angle);
	str += " mInv:" + std::to_string(massInv);
	str += " iInv" + std::to_string(inertiaInv);
	return str;
}