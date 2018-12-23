#ifndef VERSION_H
#define VERSION_H
#endif
#include<iostream>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class result_of_img{
    public:
            void setPoint(Point2f & P /*,Point2f & OP*/ )
            {
                center.x=P.x;
                center.y=P.y;
                //oldcenter.x=OP.x;
                //oldcenter.y=OP.y;
            }
            void setPoint()
            {
                center.x = 0;
                center.y = 0;
            }
            Point2f center;
            Point2f oldcenter;

};

//result_of_img result,RPoint;
//Mat frame,grayimg,fimg;
//vector<Mat> RGB;

 result_of_img & imgslover(Mat & img);
//bool sortfunction(RotatedRect & R1,RotatedRect & R2){return (R1.size.area()<R2.size.area());}
