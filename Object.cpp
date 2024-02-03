#include "Object.h"
#include "Constant.h"

float Object::getM() {
	return mass;
}

void Object::setMass(const float m) {
	mass = m;
}

Vec2 Object::getC()const {
	return center;
}

void Object::setC(const Vec2 c) {
	center = c;
}

Vec2 Object::getV()const{
	return velocity;
}

void Object::setV(const Vec2 v) {
	velocity = v;
}

void Object::setAngle(const float ang) {
	angle = ang;
}

void Object::setAngV(const float ang_v) {
	angle_v = ang_v;
}

void Object::updateAngle(const float step) {
	angle += (angle_v * step);
}

float Object::getAngle()const {
	return angle;
}

float Object::getAngV()const {
	return angle_v;
}

bool Object::isActive()const {
	return active;
}

void Object::addV(const Vec2 acc) {
	if (!active) {
		return;
	}
	velocity = acc + velocity;
}

void Object::addVang(const float acc) {
	if (!active)
	{
		return;
	}
	angle_v += acc;
}

int Object::getIndex()const {
	return index;
}

void Object::setIndex(const int i) {
	index = i;
}

uint16_t Object::getId() const{
	return id;
}

void Object::setId(const uint16_t i) {
	id = i;
}

void Object::setFri(const float fri) {
	friction = fri;
}

float Object::getFri()const {
	return friction;
}

void Object::setE(const float e_) {
	e = e_;
}

float Object::getE()const {
	return e;
}

Type Object::getType()const{
	return type;
}

const BBox& Object::getBbox()const{
	return bbox;
}

//コンストラクタ
Object::Object(Vec2 v ,Type type ,float mass, Color color , bool act , float ang , float ang_v ) 
	:velocity(v) ,  type(type),color(color) ,mass(mass),active(act) , angle(ang), angle_v(ang_v),touch(false)
{
	e = Constant::RESTITUTION;
}

void Object::setColor(Color c) {
	this->color = c;
}

unsigned int Object::getColor() const{
	return color;
}

//慣性テンソルの取得
float Object::getI()const {
	return inertiaTensor;
}

//円運動の速度を取得　point ローカル座標
//return ワールド座標での速度
Vec2 Object::getCirV(const Vec2& point)const {
	//大きさを取得 v = rω
	float length = point.norm() * angle_v;
	Vec2 result = point.normal().normalize() * length;
	return result.rotation(angle);
}

bool Object::isTouch()const {
	return touch;
}

void Object::setTouch(const bool t) {
	touch = t;
}