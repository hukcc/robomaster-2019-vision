/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */
#include "slover/armor_recorder.hpp"
#include "slover/angle_slover.hpp"
#include "utility/debug_utility.hpp"
#include "slover/kalman.hpp"

namespace autocar
{
namespace slover
{

#define show_src_rect

int count=2;
const int bullet_speed = 18;     
const cv::Point ptoffset = cv::Point(22,0); // 子弹的偏移量 offset x → y ↓  
Point2f predictPoint = Point2f(0,0);

/**
 * @brief 定义一个距离函数，计算装甲片之间姿态的“距离”
 */
inline bool pos_distance(const vision_mul::armor_pos& pos1, const vision_mul::armor_pos& last_pos )
{
    return std::sqrt((pos1.angle_x - last_pos.angle_x) * (pos1.angle_x - last_pos.angle_x) + 
                     (pos1.angle_y - last_pos.angle_y) * (pos1.angle_y - last_pos.angle_y) +
                      std::abs(pos1.angle_z - last_pos.angle_z))/10;   
}

vision_mul::armor_pos Armor_recorder::SlectFinalArmor(std::vector<vision_mul::armor_info> &armors, AngleSolver& angle_slover, AngleSolverFactory& angle_slover_factory, cv::Mat & src) 
{
    // for perdict
    
    //if(predict_flag!=0)  predictPoint = Point2f(0,0);
    std::vector<vision_mul::armor_pos> pos_vect;
#ifdef show_src_rect
    std::vector<vision_mul::armor_info> armor_vect;
#endif
    vision_mul::armor_pos armor_pos_;
    
    for (auto armor : armors)
    {
        double armor_ratio = std::max(armor.rect.size.width, armor.rect.size.height) / 
							 std::min(armor.rect.size.width, armor.rect.size.height);
        cv::RotatedRect rect = armor.rect;
        
        //
        //armor.rect = armor.rect + predictPoint - armor.rect.center;
        //armor.rect.center = predictPoint; //for predict
        RotatedRect PRrect = RotatedRect(predictPoint,rect.size,rect.angle);
        rect = PRrect;
        circle(src,rect.center,10,Scalar(0,255,0));
        //imshow("pre",src);
        if (armor_ratio < 4)
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_SAMLL_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                #ifdef show_src_rect
                armor_vect.push_back(armor);
                #endif
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        }
        else
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                #ifdef show_src_rect
                armor_vect.push_back(armor);                                    //aemor vect 就是最终装甲的vector
                #endif
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        } // if infantry or hero
    } // for

    vision_mul::armor_pos last_pos;
    
    if(history_armor.size()<2){
        history_armor.push_back(armor_vect[0]);
    }
    else{
        history_armor[count%2] = armor_vect[0];
        count++;
    }

    if (history_pos.size())
    {
        last_pos = history_pos.back();
    }
   //printf("%lf , %lf       ",armor_vect[0].rect.center.x,armor_vect[0].rect.center.y);
    //printf("%lf , %lf \n",history_armor[(count)%2].rect.center.x,history_armor[(count)%2].rect.center.y);
    if(predict_flag < 15)   //丢失目标的帧数的上限值
    predictPoint = CKalman(armor_vect[0].rect.center, src ,history_armor[(count)%2].rect.center);
//   加入对于预测点的结算    

    

    //waitKey(0);









    if(pos_vect.size())
    {
        double dis_min = 100000000;
        int idx = 0;
        for (int i = 0; i != pos_vect.size(); ++i)
        {
            double dis = pos_distance(pos_vect[i],last_pos);
            if (dis < dis_min)
            {
                dis_min = dis;
                idx = i;
            }
        }
        #ifdef show_src_rect
             draw_rotated_rect(src, armor_vect[idx].rect,cv::Scalar(0,255,255),2);
        #endif
        return pos_vect[idx];
    }
    return vision_mul::armor_pos();
}

} // namespace slover
} // namepsace autocar
