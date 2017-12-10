#ifndef Points_H
#define Points_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "spline.h"

using namespace std;
using namespace tk; // spline

class Points {

  protected:
    // define wp spline trajectory
    spline wp_spline_x;
    spline wp_spline_y;
    spline wp_spline_dx;
    spline wp_spline_dy;

  public:
    Points(){};
    Points(string Points_file_);
    ~Points() {};

    vector<double> getXY(double s, double d);

};

#endif // ROAD_H
