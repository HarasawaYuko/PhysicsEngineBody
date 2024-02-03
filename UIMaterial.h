#pragma once
#include "include.h"
#include "Math.h"
#include <functional>

bool isIn(const int, const int, const int, const int);

//�F
typedef unsigned int Color;
const Color COLOR_GREEN = GetColor(10, 230, 20);
const Color COLOR_BLUE = GetColor(1, 20, 150);
const Color COLOR_LBLUE = GetColor(101, 187, 233);
const Color COLOR_ORANGE = GetColor(0xFF, 0x99, 0x33);
const Color COLOR_WHITE = GetColor(230 , 230 ,230);
const Color COLOR_BLACK = GetColor(20, 20, 20);
const Color COLOR_RED = GetColor(230, 10, 10);
const Color COLOR_GRAY = GetColor(120, 120, 120);
const Color COLOR_YELLOW = GetColor(255 , 217 ,0);
const Color COLOR_SBLUE = GetColor(20 ,200 , 200);
const Color COLOR_PURPLE = GetColor(230 , 10 , 255);
const Color COLOR_LRED = GetColor(255, 80, 80);

const Color colors[] = {COLOR_GREEN ,
						COLOR_BLUE,
						COLOR_SBLUE,
						COLOR_ORANGE,
						COLOR_RED,
						COLOR_YELLOW,
						COLOR_PURPLE,
						COLOR_LRED};

class Button {
private:
	//�摜
	int pic;//�ʏ�̉摜
	int onPic;//�J�[�\���������Ă��鎞�̉摜
	int offPic;//�������̉摜
	//����
	int sound;
	//����̓_
	int x;
	int y;
	//�傫��
	int width;
	int height;
	bool isOn;//�J�[�\���������Ă��邩
	bool active = true;//�{�^�����L����
	bool push = false;//�����ꂽ��

public:
	Button() {};
	Button(const int pic, const int onPic, const int sound ,const int x, const int y, const int width , const int height , const int offPic = -1);
	Button(const int pic, const int onPic, const int sound, const int y, const int width ,const int offPic = -1);
	void update();
	void draw();
	void act(const bool);
	bool isPush();
	void finalize();
};

class RadioButton {
private:
	int num;
	int x;
	int y;
	int space;
	int size;
	int fontsize;
	
	std::vector<std::string> strs;
	int select;
	int on;
public:
	RadioButton(){}
	RadioButton(const int , const int , const int ,const int, const int ,const int,const std::vector<std::string>);
	void update(bool*);
	void draw()const;
	int getSelect() const;
};

class SlideBar {
private:
	int length;
	int x;
	int y;
	int size;
	int selectX;
	bool on;
public:
	SlideBar(){}
	SlideBar(const int , const int , const int , const int);
	void update(bool*);
	void draw() const;
	float getRate()const;
};

//Y�����]�̕`��֐�
int DrawCircleP(const float , const float , const float , const unsigned int, const int FillFrag = 1, const float LineThickness = 1.0f);
int DrawBoxP(const Vec2 cen, const int , const int , const double angle, const int graph, const float h, const float w, const int size_x, const int size_y, const int transFlag = 0, const int turnFlag = 0);
int DrawSegment(const Segment& , const unsigned int);
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int);
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int , const float thick);
int DrawPoint(const Vec2& , const unsigned int);
int DrawTriP(const Vec2& ,const Vec2& , const Vec2& ,const unsigned int color = COLOR_BLACK , const int fill = 1 , const float thick = 1.f);
int DrawStrP(const Vec2& , const std::string , const unsigned int color = COLOR_BLACK);

//float�l�̌����ϊ�
std::string FtoStr(const float);

//�����_���ŐF���擾
unsigned int getColorRand();