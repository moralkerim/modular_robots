#include "lpf.hpp"

lpf::lpf()  {

}


float lpf::Run(float x) {


	float y = 0.8544*y_ + 0.07282 * x + 0.07282 * x_;
	x_ = x;
	y_ = y;
	return y;

}

lpf::~lpf() {}
