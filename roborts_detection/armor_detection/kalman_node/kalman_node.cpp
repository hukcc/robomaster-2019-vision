#include "kalman_node.h"
    // int filter_cnt = 0;
    int loop_cnt   = 0;
    cv::RotatedRect hisRect;
    cv::RotatedRect hisprerect;
    float his_vx =0;
    float max_vel_x =0;
    float op_max_vel_x =0;
    float his_max_vel_x =0;
    float his_vel_x = 0;
    std::vector<float> Pre_vels_(9);
    std::vector<float> Pre_vels_copy(9);
using namespace cv;

  bool comp(const float &a, const float &b)
  {
      return abs(a) > abs(b);
  }

  float kalman_node::PreFilter(float Pre_vel)
  {
    // Pre_vels_[filter_cnt%3] = Pre_vel;
    Pre_vels_[0] = Pre_vels_[1];
    Pre_vels_[1] = Pre_vels_[2];
    Pre_vels_[2] = Pre_vels_[3];
    Pre_vels_[3] = Pre_vels_[4];
    Pre_vels_[4] = Pre_vels_[5];
    Pre_vels_[5] = Pre_vels_[6];
    Pre_vels_[6] = Pre_vels_[7];
    Pre_vels_[7] = Pre_vels_[8];
    Pre_vels_[8] = Pre_vel;

    Pre_vels_copy = Pre_vels_;
    sort(Pre_vels_copy.begin(), Pre_vels_copy.end(), comp);
    // ROS_ERROR("%d", filter_cnt%9);
    // ROS_ERROR("%f", Pre_vels_copy[0]);
    // ROS_ERROR("%f", Pre_vels_copy[1]);
    // ROS_ERROR("%f", Pre_vels_copy[2]);
    // ROS_ERROR("%f", Pre_vels_copy[3]);
    // ROS_ERROR("%f", Pre_vels_copy[4]);
    // ROS_ERROR("%f", Pre_vels_copy[5]);
    // ROS_ERROR("%f", Pre_vels_copy[6]);
    // ROS_ERROR("%f", Pre_vels_copy[7]);
    // ROS_ERROR("%f", Pre_vels_copy[8]);
    // ROS_ERROR("--------------------------------------------------------------------");
    // ROS_ERROR("%f", Pre_vels_[0]);
    // ROS_ERROR("%f", Pre_vels_[1]);
    // ROS_ERROR("%f", Pre_vels_[2]);
    // ROS_ERROR("%f", Pre_vels_[3]);
    // ROS_ERROR("%f", Pre_vels_[4]);
    // ROS_ERROR("%f", Pre_vels_[5]);
    // ROS_ERROR("%f", Pre_vels_[6]);
    // ROS_ERROR("%f", Pre_vels_[7]);
    // ROS_ERROR("%f", Pre_vels_[8]);
    // ROS_ERROR("=====================================================================");
    // filter_cnt++;
    // if(Pre_vel > 0 )
    return Pre_vels_copy[4];
    // else
    // return -Pre_vels_copy[4];
    
    
  }

  cv::RotatedRect kalman_node::KF(cv::RotatedRect  armor_rect)
  {
    //armor_rect = armor_rect; //init
     //handwrite filter
      vx_gimbal = gimbal_info_.yaw_rate; //gimbal_info_.yaw_rate;
      vy_gimbal = gimbal_info_.pitch_rate;
      y_gimbal =gimbal_info_.pitch_ecd_angle;
      p_gimbal =gimbal_info_.yaw_ecd_angle;
      

      if(y_gimbal - his_y_gimbal < 0) vx_gimbal = -vx_gimbal;
      if(p_gimbal - his_p_gimbal < 0) vy_gimbal = -vy_gimbal;


      his_p_gimbal = p_gimbal;
      his_y_gimbal = y_gimbal;


      if(loop_cnt <  1 )  //might useless?
      hisRect = armor_rect;
      if(loop_cnt > 30)
      {
        loop_cnt = 0;
        max_vel_x = 0;
        op_max_vel_x = 0;
        his_max_vel_x = 0;
      }
      
      loop_cnt++;

      if((his_vx > 0 && -(hisRect.center.x - armor_rect.center.x)*1.5 < 0) || (his_vx < 0 && -(hisRect.center.x - armor_rect.center.x)*1.5 > 0) )
      return armor_rect;/* code */
     //kalman predict
        //kalman filter init
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
    vel_x = -(hisRect.center.x - armor_rect.center.x)*2.4;
    vel_y = -(hisRect.center.y - armor_rect.center.y)*0.8;

    state.at<float>(0) = armor_rect.center.x;   //x
    state.at<float>(1) = armor_rect.center.y;   //y
    state.at<float>(2) = hisRect.center.x - armor_rect.center.x;   //vx;
    state.at<float>(3) = hisRect.center.y - armor_rect.center.y;   //vy;
    
    int temp_vel = PreFilter(vel_x);
    if(temp_vel > max_vel_x)
    {
      max_vel_x = temp_vel;
      
    }
    
    if(temp_vel < op_max_vel_x)
    op_max_vel_x = temp_vel;

    
    if (his_vel_x != 0 && ((his_vel_x > 0 && vel_x < 0) || (his_vel_x < 0 &&  vel_x > 0) ) ) {
        vel_x = his_vel_x;
      }
    if (vel_x >150) {
        vel_x = 0;
    }
    

    if (vel_x > 2) {
      vel_x = max_vel_x;
      his_max_vel_x = vel_x;
    }
    if (vel_x < -2)
    {
      vel_x = op_max_vel_x;
      his_max_vel_x = vel_x;
    }
    else
    {
      vel_x = his_max_vel_x;
    }
    
    
    

    // ROS_ERROR("%f", vel_x);
    KF.statePost.at<float>(0) = armor_rect.center.x;//x;
    KF.statePost.at<float>(1) = armor_rect.center.y;                    //y;
    KF.statePost.at<float>(2) = vel_x*1;  //this number is a contrl param to control the power of predict   // + vx_gimbal;                    //vx;
    KF.statePost.at<float>(3) = vel_y;   // + vy_gimbal;                    //vy;

    his_vel_x = vel_x;

    Point statePt = Point(state.at<float>(0), state.at<float>(1));                                                                          //这边这个状态向量 1.角度 2.速度（估计是）
    Mat prediction = KF.predict();      //调用预测函数 将返回的先验估计值储存在prediction
    double px = prediction.at<float>(0);  //上面返回的先验预测向量中的第一个元素作为预测的角度
    double py = prediction.at<float>(1);
    Point predictPt = (Point2f(px,py)); //change x for miao zhun

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

    PreRect = RotatedRect(predictPt,armor_rect.size,armor_rect.angle);
    //hisInfo
    hisRect = armor_rect;
    his_vx  = -(hisRect.center.x - armor_rect.center.x)*1.8;
    hisprerect = PreRect;
    // armor_rect = armor_rect; 

    return PreRect; //output
  }



