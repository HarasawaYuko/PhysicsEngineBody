#pragma once
#include "Object.h"
#include "Convex.h"
#include "Circle.h"

namespace Detect {
	//�u���[�h�t�F�[�Y
	bool broard(const Object* ,const Object*);

	//�i���[�t�F�[�Y �}�`�ɂ���ăI�[�o�[���[�h
	bool narrow(Convex* , Convex* , float* , Vec2* , Vec2* , Vec2* );
	bool narrow(Circle* , Convex* , float* , Vec2* , Vec2* , Vec2* );
	bool narrow(Circle* , Circle* , float* , Vec2* , Vec2* , Vec2* );

}