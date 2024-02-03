#include "UIMaterial.h"
#include "Mouse.h"
#include "Rand.h"

bool isIn(const int x, const int y, const int width, const int height) {
	if (x < Mouse::instance()->getX() && Mouse::instance()->getX() < x + width
		&& (y - height) < Mouse::instance()->getY() && Mouse::instance()->getY() < y){
		return true;
	}
	else {
		return false;
	}
}

/*****�{�^���N���X******/

//�S�Ă��w�肷��R���X�g���N�^
Button::Button(const int pic , const int onPic , const int sound ,const int x ,const int y , const int width, const int height, const int offPic)
	: pic(pic) ,onPic(onPic),sound(sound), x(x), y(y), width(width), height(height), isOn(false), offPic(offPic)
{}

//�����y�ƕ����w�肷��@�����z�u
Button::Button(const int pic, const int onPic, const int sound, const int y, const int width, const int offPic)
	: pic(pic), onPic(onPic), sound(sound), y(y), width(width) , isOn(false), offPic(offPic)
{
	//x�̒l��ݒ�
	x = (WIN_SIZE_X / 2) - (width / 2);
	//�摜�T�C�Y���擾
	int x_size;
	int y_size;
	GetGraphSize(pic , &x_size ,&y_size);
	//height�̐ݒ�
	float rate = (float)width / (float)x_size;
	height = (int)((float)y_size * rate);
}

void Button::update() {
	if (!active) return;
	push = false;
	isOn = false;
	if (isIn(x, y, width, height)) {
		isOn = true;
		if (Mouse::instance()->getClickNow(LEFT_CLICK)) {
			push = true;
		}
	}
}

void Button::draw() {
	if (!active) {
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), offPic, true);
		return;
	}
	if(isOn){
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), onPic ,true);
	}
	else{
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), pic, true);
	}
	//�����ꂽ�特�����Đ�
	if (push) {
		PlaySoundMem(sound , DX_PLAYTYPE_BACK, true);
	}
}

void Button::act(const bool a) {
	active = a;
}

bool Button::isPush() {
	return push;
}

void Button::finalize() {
	//�摜�̍폜
	if(pic != -1)DeleteGraph(pic);
	if(onPic != -1)DeleteGraph(onPic);
	if(offPic != -1)DeleteGraph(offPic);
	//�������Đ������m�F����
	while (CheckSoundMem(sound) == 1);
	if(sound != -1)DeleteSoundMem(sound);
}

/**********���W�I�{�^��***********/

RadioButton::RadioButton(const int x , const int y ,const int size , const int space, const int num ,const int font ,const std::vector<std::string> strs)
	:x(x) , y(y) ,size(size),space(space), num(num), on(-1), fontsize(font), strs(strs)
{}

void RadioButton::draw() const{
	SetFontSize(fontsize);
	//num�J��Ԃ�
	for (int i = 0; i < num; i++) {
		DrawCircle(x + size + i*space , y + size,size, COLOR_WHITE, true);
		DrawString(x + size*2 + i*space + 10, y , strs[i].c_str(),COLOR_BLACK );
	}
	if (on >= 0) {
		DrawCircle(x + size + on * space, y + size , size, COLOR_LBLUE , false , 5);
	}
	//�I���ӏ��\��
	DrawCircle(x + size + select * space, y + size, (int)(size * 0.6), COLOR_BLUE, true);
}

int RadioButton::getSelect() const {
	return select;
}

/******�X���C�h�o�[*******/
SlideBar::SlideBar(const int x , const int y , const int length ,const int size ) 
	:x(x) , y(y) , length(length) , size(size) , on(false), selectX(x)
{}

void SlideBar::update(bool*click) {
	*click = false;
	on = false;
	if (isIn(x , y - size ,length , y+size*2)) {
		on = true;
		if (Mouse::instance()->getClick(LEFT_CLICK)) {
			*click = true;
			selectX = (int)Mouse::instance()->getX();
		}
	}
}

void SlideBar::draw() const{
	DrawBox(x, y, x + length, y + 6 , COLOR_GRAY, true);
	//�I���ӏ��\��
	DrawCircle(selectX, y + 3 , size, COLOR_BLUE, true);
	if (on) {
		DrawCircle(selectX, y+3 , size , COLOR_LBLUE, false, 3);
	}
}

float SlideBar::getRate()const {
	return ((float)(selectX - x)/length);
}

//y���𔽓]�������~�`��֐�
int DrawCircleP(const float x, const float y, const float r , const unsigned int Color , const int FillFrag , const float LineThickness) {
	return DrawCircleAA(x , (float)(WIN_SIZE_Y - y) , r , (int)r, Color, FillFrag, LineThickness);
}

//y���𔽓]�����������`�`��֐�
int DrawBoxP(const Vec2 cen ,const int cen_r_x , const int cen_r_y ,const double angle ,const int graph , const float h , const float w , const int size_x ,const int size_y,const int transFlag , const int turnFlag) {
	//�摜�̒��S���W���擾
	double exRateX = (double)w/(double)size_x;
	double exRateY = (double)h/(double)size_y;
	return DrawRotaGraph3((int)cen.x , WIN_SIZE_Y - (int)cen.y , cen_r_x , cen_r_y ,exRateX , exRateY , angle , graph , transFlag , turnFlag);
}

//y���𔽓]�������O�p�`�̕`��֐�
int DrawTriP(const Vec2& p0, const Vec2& p1, const Vec2& p2,const unsigned int color , const int fill ,const float thick) {
	return DrawTriangleAA(p0.x , WIN_SIZE_Y -  p0.y , p1.x , WIN_SIZE_Y - p1.y ,p2.x ,  WIN_SIZE_Y - p2.y , color , fill ,thick);
}

//�����̕`��
int DrawSegment(const Segment& line, const unsigned int color) {
	return DrawLineAA(line.start.x , WIN_SIZE_Y - line.start.y , line.end.x , WIN_SIZE_Y - line.end.y , color , 3.f);
}

//�����̕`��@��_���󂯎��I�[�o�[���[�h
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int color) {
	return DrawLineAA(s.x , WIN_SIZE_Y - s.y , e.x , WIN_SIZE_Y - e.y , color ,3.f);
}

//�����̕`��@��_���󂯎��I�[�o�[���[�h
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int color , const float thick) {
	return DrawLineAA(s.x, WIN_SIZE_Y - s.y, e.x, WIN_SIZE_Y - e.y, color, thick);
}

int DrawPoint(const Vec2& v, const unsigned int color) {
	return DrawCircle((int)v.x , WIN_SIZE_Y - (int)v.y , 5 , color);
}

int DrawStrP(const Vec2& p , const std::string str , const unsigned int color) {
	return DrawString((int)p.x ,WIN_SIZE_Y - (int)p.y , str.c_str(), color);
}

std::string FtoStr(const float val) {
	char tmp[255];
	sprintf_s(tmp, "%.0f", val);
	return std::string(tmp);
}

unsigned int getColorRand() {
	Rand* rand = Rand::instance();
	int a = rand->getI(0, 7);
	return colors[a];
}