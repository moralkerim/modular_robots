#include "lpf.hpp"

lpf::lpf(double a, double b, double c)  {
	a_f = a;
	b_f = b;
	c_f = c;
}


float lpf::Run(float x) {


	//float y = 0.8544*y_ + 0.07282 * x + 0.07282 * x_;
	float y = a_f*y_ + b_f * x + c_f * x_;
	x_ = x;
	y_ = y;
	return y;

}

lpf::~lpf() {}
