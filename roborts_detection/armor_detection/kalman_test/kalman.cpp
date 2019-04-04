#include "kalman.h"
cv::RotatedRect hisRect;
int loop_cnt = 0;
double y_gimbal =0;
double p_gimbal =0;
double his_y_gimbal =0;
double his_p_gimbal =0;
double vx_gimbal =0;
double vy_gimbal =0;
kalman KLfilter;
using namespace cv; 

void kalman::angleCallback(const roborts_msgs::GimbalInfo::ConstPtr& msg)
{
  KLfilter.gimbal_info_.mode             = msg->mode;
  KLfilter.gimbal_info_.pitch_ecd_angle  = msg->pitch_ecd_angle;
  KLfilter.gimbal_info_.yaw_ecd_angle    = msg->yaw_ecd_angle;
  KLfilter.gimbal_info_.pitch_gyro_angle = msg->pitch_gyro_angle;
  KLfilter.gimbal_info_.yaw_gyro_angle   = msg->yaw_gyro_angle;
  KLfilter.gimbal_info_.yaw_rate         = msg->yaw_rate;
  KLfilter.gimbal_info_.pitch_rate       = msg->pitch_rate;
  // ROS_ERROR("gimbal_info_.mode: %d ", gimbal_info_.mode );
  
  // ROS_ERROR("gimbal_info_.pitch_ecd_angle: %lf ",  gimbal_info_.pitch_ecd_angle);
  // ROS_ERROR("gimbal_info_.yaw_ecd_angle: %lf ",    gimbal_info_.yaw_ecd_angle );
  // ROS_ERROR("gimbal_info_.pitch_gyro_angle: %lf ", gimbal_info_.pitch_gyro_angle);
  // ROS_ERROR("gimbal_info_.yaw_gyro_angle: %lf ",   gimbal_info_.yaw_gyro_angle);
  // ROS_ERROR("gimbal_info_.yaw_rate: %lf ",         gimbal_info_.yaw_rate);
  // ROS_ERROR("gimbal_info_.pitch_rate: %lf ",       gimbal_info_.pitch_rate);
  ROS_ERROR("ok?");
  
}

void kalman::getAngle()
{
    sub = gimbal_nh_.subscribe("gimbal_info", 100, &kalman::angleCallback, this);
    
}

cv::RotatedRect CKalman(RotatedRect armor_rect )
{
    
    KLfilter.getAngle();
    vx_gimbal = KLfilter.gimbal_info_.yaw_rate;
    vy_gimbal = KLfilter.gimbal_info_.pitch_rate;
    y_gimbal =KLfilter.gimbal_info_.pitch_ecd_angle;
    p_gimbal =KLfilter.gimbal_info_.yaw_ecd_angle;

    if(y_gimbal - his_y_gimbal < 0) vx_gimbal = -vx_gimbal;
    if(p_gimbal - his_p_gimbal < 0) vy_gimbal = -vy_gimbal;


    his_p_gimbal = p_gimbal;
    his_y_gimbal = y_gimbal;


    if(loop_cnt <  1 )
    {
        hisRect = armor_rect;
        
    }
    
    loop_cnt++;

    KalmanFilter KF(4, 2);       //三个参数 1.过程状态向量维度 2.观测向量维度  3.控制向量的维度
    Mat state(4, 1, CV_32F); /* (phi, delta_phi) */     //创建了一个2×1的向量 用来描述状态
    Mat processNoise(4, 1, CV_32F);                     //2*1 的矩阵用于描述系统噪声
    Mat measurement = Mat::zeros(2, 1, CV_32F);         //当前观测矩阵
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                0, 1, 0, 1,
                                                0, 0, 1, 0,
                                                0, 0, 0, 1 );    //对于状态转移矩阵赋初值
    setIdentity(KF.measurementMatrix);  //对于观测矩阵设置单元矩阵  （就是单位矩阵）主对角线都是一
    //这里后面加上一个参数 代表的是用后面这个参数的值替换对角线上的所有值       //注意这里scalar：：all是将三个通道都设置为这个值  由此可以推知这些矩阵每一个元素也是包含三通道的
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     //初始化系统噪声协防差矩阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); //测量误差
    setIdentity(KF.errorCovPost, Scalar::all(1));           //先验误差？
    //初始化运动状态
    state.at<float>(0) = armor_rect.center.x;   //x
    state.at<float>(1) = armor_rect.center.y;   //y
    state.at<float>(2) = hisRect.center.x - armor_rect.center.x;   //vx;
    state.at<float>(3) = hisRect.center.y - armor_rect.center.y;   //vy;
    
    KF.statePost.at<float>(0) = armor_rect.center.x;//x;
    KF.statePost.at<float>(1) = armor_rect.center.y;                    //y;
    KF.statePost.at<float>(2) = 100*vx_gimbal;//-(hisRect.center.x - armor_rect.center.x)*2;   // + vx_gimbal;                    //vx;
    KF.statePost.at<float>(3) = 100*vy_gimbal;//-(hisRect.center.y - armor_rect.center.y)*2;   // + vy_gimbal;                    //vy;
    
    //std::cout << -(hisRect.center.x - armor_rect.center.x)*2 <<"!!!!!!"<< 100*vx_gimbal << "\n" ;
    // ROS_ERROR("gimbal_info_.yaw_rate: %lf ",KLfilter.gimbal_info_.yaw_rate);
    // ROS_ERROR("gimbal_info_.pitch_rate: %lf ",KLfilter.gimbal_info_.pitch_rate);

    Point statePt = Point(state.at<float>(0), state.at<float>(1));                                                                          //这边这个状态向量 1.角度 2.速度（估计是）
    Mat prediction = KF.predict();      //调用预测函数 将返回的先验估计值储存在prediction
    double px = prediction.at<float>(0);  //上面返回的先验预测向量中的第一个元素作为预测的角度
    double py = prediction.at<float>(1);
    //Point predictPt = (Point2f(calcPoint(center, px, py)) + 0*oldpoint);   //用预测出来的角度求出那个点的位置
    Point predictPt = (Point2f(px,py));

    randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));  //这里应该是考虑了误差 参生了一个随机的误差矩阵 后面要加在先验向量上

    // generate measurement
    measurement += KF.measurementMatrix*state;      //左乘观测矩阵 将状态向量还原为观测向量

    double mx = measurement.at<float>(0);
    double my = measurement.at<float>(1);
    Point measPt = Point(mx,my);
    if(theRNG().uniform(0,4) != 0)      //如果      就更新一次预测的值
        KF.correct(measurement);

    randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
    state = KF.transitionMatrix*state + processNoise;       //更新下一次的输入值

    //armor_rect.center = predictPt;
    cv::RotatedRect PreRect = RotatedRect(predictPt,armor_rect.size,armor_rect.angle);
    hisRect = armor_rect;
    return PreRect;
}



