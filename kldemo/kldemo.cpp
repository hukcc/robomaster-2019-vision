#include "kldemo.hpp"


Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )      //鼠标操作回调函数
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}

// VideoCapture cap;       //声明相机对象
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);   //声明对象并且用构造函数初始化 这里是迭代函数的限制条件（1.类型 2.最大迭代次数 3.结果的准确度）
    Size subPixWinSize(10,10), winSize(31,31);  //不知道是干啥的

    const int MAX_COUNT = 500;      //迭代器最大循环
    bool needToInit = true;       //全局初始化控制条件
    
    
    // cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    // string input = parser.get<string>("@input");

    // if( input.size() == 1 && isdigit(input[0]) )
    //     cap.open(input[0] - '0');
    // else
    //     cap.open(input);

    
    //cv::namedWindow( "LK Demo", 1 );
    //cv::setMouseCallback( "LK Demo", onMouse, 0 );

    Mat gray, prevGray, image;//, frame;
    vector<Point2f> points[2];  //一个动态数组的数组



int kldemo(Mat & frame)


{
    

    //for(;;)
    //{   
        if( frame.empty() ){        //做一个异常抛出
            return 1;
        } 
        else{
            imshow("",frame);
        }
        
        frame.copyTo(image);        //复制图像进行操作
        cvtColor(image, gray, COLOR_BGR2GRAY);      //转化为灰度图像
        
        if( needToInit )    //如果需要全局角点初始化    这里只会初始化一次 在运行完一个循环之后就会关闭needtoinit条件
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);    //对于图像或者指定的部分进行角点检测    这里用的其实是harris角点检测
            //                输入图像  存角点的容器 最大角点数目 角点质量 间距 掩模
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);    //亚像素角点检测 目的是对于上面的角点检测作进一步的优化
            addRemovePt = false;
        }
        else if( !points[0].empty() )   //如果不是第一轮检测 或者说如果上一轮中有遗留的检测到的角点
        {
            vector<uchar> status;
            vector<float> err;          //这两个动态数组不知道是干啥用的
            if(prevGray.empty())
                gray.copyTo(prevGray);  //如果pregray是空的就把当前的灰度图烤给它
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }

        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char)waitKey(10);
        if( c == 27 )
            return 1;
        switch( c )
        {
        case 'r':
            needToInit = true;
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            break;
        
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    //}

    return 0;
}
