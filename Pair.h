#pragma once
#include "Math.h"
#include "Object.h"
#include "Collision.h"

/*
*ペアクラス
* ブロードフェーズを実装するまでは使用しない
*/
enum Kind : uint8_t {
	CIRCLE_CIRCLE = 0b0000'0001,
	CIRCLE_CONVEX = 0b0000'0011,
	CONVEX_CONVEX = 0b0000'0010
};

enum PairType {
	New,
	Keep
};

class Pair {
private:
	Object* obj[2];//オブジェクトへのアクセス
	Kind kind;//オブジェクトの組み合わせ
	PairType type;//新規に見つかったペアか
	uint32_t key;//ユニークなキー
	Collision* collision;//衝突情報
public:
	Pair(Object* , Object*);
	Kind getKind()const;
	PairType getType()const;
	uint32_t getKey()const;
	void setType(PairType);
	Object* getObj0()const;
	Object* getObj1()const;
	Collision* getCol()const;
	void refreshCp();//contactPointをリフレッシュする
	std::string toString()const;
};