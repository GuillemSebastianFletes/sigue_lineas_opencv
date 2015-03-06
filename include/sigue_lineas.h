#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/highgui.h>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <sstream>

using namespace cv;
using namespace std;

class sigue_lineas
{
public:

    ~sigue_lineas();
    void follow();
    void Threshold();
    void roi(int, int, int, int, int, int,int, int);
    Point find_center_line(Point);
    Point find_center_curve(Point);

    Mat linea, curva;
    cv::Mat camera;
};
