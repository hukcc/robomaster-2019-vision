#ifndef KALMAN_H
#define KALMAN_H
#endif
#include<iostream>
#include<opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include "io/io.h"
#include "state/error_code.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

#include "roborts_msgs/GlobalPlannerAction.h"
#include "roborts_msgs/LocalPlannerAction.h"

#include "roborts_msgs/GimbalInfo.h"

using namespace cv;
using namespace std;

class kalman
{
private:
    //RotatedRect hisRect;
    Point2f point4kalman;
    Point2f oldpoint4kalman;
    Point2f oldpoint;
    
    ros::NodeHandle gimbal_nh_;
    ros::Subscriber sub;    

public:
    void getAngle();
    roborts_msgs::GimbalInfo gimbal_info_;
    void angleCallback(const roborts_msgs::GimbalInfo::ConstPtr& msg);

    

};


cv::RotatedRect CKalman(cv::RotatedRect armor_rect);