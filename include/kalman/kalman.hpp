#ifndef KALMAN_H
#define KALMAN_H
#endif
#include "version.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
//#include <opencv2/opencv.hpp>
//#include <stdio.h>

Point2f CKalman(Point2f & point4kalman , Mat & img , Point2f & oldPoint4Kalman);

void CKalman();