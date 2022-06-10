#pragma once
#include <stdio.h>


class lpf {

    private:
        float x_, y_;
        double a_f, b_f, c_f;

    public:
        float y;

    public:
        lpf(double a, double b, double c);
        float Run(float x);
        ~lpf();


};


