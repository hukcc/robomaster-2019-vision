#include "slover/kalman.hpp"
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

float history_vx=0;
int his_flag=0;
Mat history_mat;
vector<Mat> history_mats(2);
Mat match_mat(640-3+1,480-3+1,CV_32FC1);
double minv,maxv;
Point minpoint,maxpoint;
void CKalman() {};

Point2f oldpoint = Point2f(0,0);

static inline Point calcPoint(Point2f center, double  x, double  y,int K = 1)
{
    return  Point2f(x,y);       //这是用来计算圆周上的点的函数
}



//float x=250,y=250,vx=1,vy=-1;


Point2f CKalman(Point2f & point4kalman , Mat & img ,Point2f & oldPoint4Kalman , int & predict_count , int & dis)
{

    if(his_flag>1){
        
    imshow("img",img);
    //waitKey(0);
    Rect roi(319,25,30,30);
    rectangle(img,roi,Scalar(255,0,0));
    history_mat = history_mats[(his_flag-1)%2];
    Mat tempmat = history_mat(roi);
    imshow("his",history_mat);
//////////////////////////////////////////////////////////////////////////////////
    vector<Mat> RGB;
    split(history_mat,RGB);
///////////////////////////////////////surf///////////////////////////////////////
    /*
    cvtColor(history_mat,history_mat,COLOR_RGB2GRAY);
    vector<KeyPoint> train_keypoint;
    Mat train_descriptor;
    */
    
    





//////////////////////////////////////////////////////////////////////////////////
    matchTemplate(img,tempmat,match_mat,TM_CCOEFF_NORMED);
    normalize(match_mat,match_mat,0,1,NORM_MINMAX,-1,Mat());
    
    cv::minMaxLoc(match_mat,&minv,&maxv,&minpoint,&maxpoint);

    Point2f matchpoint = maxpoint;
    circle(img,matchpoint,10,Scalar(0,0,255));
    line(match_mat,matchpoint,Point2f(320,76),Scalar(0,0,255));
    
    printf("%lf\n",matchpoint.x-319);
    imshow("",match_mat);
    waitKey(0);
    
    //FLANN特征点匹配
    
    
    
    }
    
    //Mat img(500, 500, CV_8UC3);     //定义背景图像
    
    KalmanFilter KF(4, 2);       //三个参数 1.过程状态向量维度 2.观测向量维度  3.控制向量的维度
    Mat state(4, 1, CV_32F); /* (phi, delta_phi) */     //创建了一个2×1的向量 用来描述状态
    Mat processNoise(4, 1, CV_32F);                     //2*1 的矩阵用于描述系统噪声
    Mat measurement = Mat::zeros(2, 1, CV_32F);         //当前观测矩阵
    char code = (char)-1;                               //这里是标志位

    //初始化运动状态
    float x = point4kalman.x;
    float y = point4kalman.y;
    float vx = -(oldPoint4Kalman.x - point4kalman.x)*4450/dis;
    float vy = -(oldPoint4Kalman.y - point4kalman.y)*4450/dis;

    state.at<float>(0) = x;
    state.at<float>(1) = y;
    state.at<float>(2) = vx;
    state.at<float>(3) = vy;


    //for(;;)
    
        

        KF.statePost.at<float>(0) = x;
        KF.statePost.at<float>(1) = y;
        KF.statePost.at<float>(2) = vx;
        KF.statePost.at<float>(3) = vy;
        //randn( state, Scalar::all(0), Scalar::all(1) );       //opencv：：randn函数是用于 生成随机数 后面的两个参数是用来限制范围的   这里是用随机生成的数填充这个状态矩阵
        //state.at<float>(0) = 20;
        //state.at<float>(1) = 90;
        KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                    0, 1, 0, 1,
                                                    0, 0, 1, 0,
                                                    0, 0, 0, 1 );    //对于状态转移矩阵赋初值

        setIdentity(KF.measurementMatrix);  //对于观测矩阵设置单元矩阵  （就是单位矩阵）主对角线都是一
        
        //这里后面加上一个参数 代表的是用后面这个参数的值替换对角线上的所有值       //注意这里scalar：：all是将三个通道都设置为这个值  由此可以推知这些矩阵每一个元素也是包含三通道的
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     //初始化系统噪声协防差矩阵
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); //测量误差
        setIdentity(KF.errorCovPost, Scalar::all(1));           //先验误差？

        Point predictPt;

        //randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));  //后验更新值？
        //以上是初始化 每一次测量和估计开始前都要重新初始化
        
        //Point2f point_into = Point2f(1500,1500) ;
        //for(int i=0;i<3 ;i++) //内层for循环表示对于当前这一帧的图像开始进行测量
        
        //{

            

            Point2f center(img.cols*0.5f, img.rows*0.5f);   //定义出图像的中心点
            circle(img,center,2,Scalar(0,255,0));
            //float R = img.cols/3.f;     //声明圆的半径
            //double x = x;
            //double y = y;
            
            //double stateAngle = state.at<float>(0); //选取state矩阵中的第一个元素作为圆的角度 又state这玩意儿是随机填充的 所以这里的角度是随机生成的
            Point statePt = calcPoint(center, state.at<float>(0), state.at<float>(1));       //算出当前这个点的位置
            //printf("%d,%d\n",statePt.x,statePt.y);                                                                                        //这边这个状态向量 1.角度 2.速度（估计是）
            Mat prediction = KF.predict();      //调用预测函数 将返回的先验估计值储存在prediction
            double px = prediction.at<float>(0);  //上面返回的先验预测向量中的第一个元素作为预测的角度
            double py = prediction.at<float>(1);
            predictPt = (Point2f(calcPoint(center, px, py)) + 0*oldpoint);   //用预测出来的角度求出那个点的位置
//下面是更新观测值
            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));  //这里应该是考虑了误差 参生了一个随机的误差矩阵 后面要加在先验向量上

            // generate measurement
            measurement += KF.measurementMatrix*state;      //左乘观测矩阵 将状态向量还原为观测向量

            double mx = measurement.at<float>(0);
            double my = measurement.at<float>(1);
            Point measPt = calcPoint(center, mx, my);     //同样是算出观察到的点      //我觉得如果这里没有上面写入的随机误差 那么观察到的点应该和实际的点是一样的
            oldpoint = measPt;
            // plot points//画X 标记点的位置
            #define drawCross( center, color, d )                                        \
                line( img, Point( center.x - d, center.y - d ),                          \
                             Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
                line( img, Point( center.x + d, center.y - d ),                          \
                             Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )

            circle(img,predictPt,10,Scalar(0,255,0));
            //img = Scalar::all(0);
            drawCross( statePt, Scalar(255,0,0), 3 ); //blue
            drawCross( measPt, Scalar(0,0,255), 3 );    //red
            drawCross( predictPt, Scalar(0,255,0), 3 );     //green
            line( img, statePt, measPt, Scalar(0,0,255), 3, LINE_AA, 0 );
            line( img, statePt, predictPt, Scalar(0,255,255), 3, LINE_AA, 0 );

            if(theRNG().uniform(0,4) != 0)      //如果      就更新一次预测的值
                KF.correct(measurement);

            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
            state = KF.transitionMatrix*state + processNoise;       //更新下一次的输入值
            //point_into = - Point2f(state.at<float>(0),state.at<float>(1));

            imshow( "Kalman", img );
            //waitKey(30);
           
            //if( code > 0 )
              //  break;
        
        //if( code == 27 || code == 'q' || code == 'Q' )
            //break;
        //}


        
        history_mats[his_flag%2] = img;
        his_flag++;
        

        //rectangle(img,roi,Scalar(0,255,0));
        //history_mat = img(roi);
        
        
        
    if((abs(predictPt.x - point4kalman.x)>100) || (abs(predictPt.y - point4kalman.y)>50)) return point4kalman  ; 
    if(predict_count < 8) return point4kalman;
    if(abs(predictPt.y - point4kalman.y)<10 || abs(predictPt.x - point4kalman.x)<10) return point4kalman;
    
    return predictPt    ;
}