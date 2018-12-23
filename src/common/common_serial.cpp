/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <chrono>
#include <thread> 
#include "common/common_serial.h"

namespace autocar
{
namespace serial_mul
{

#define show_serial_listen
#define show_serial_publish

CLinuxSerial serial(0);
w_of_ptz ptz_w; //ptz_omiga

int32_t get_yawv() 
{
    int32_t a = ptz_w.Yaw_v;
    return a;
}
int32_t get_pitchv() 
{
    int32_t b = ptz_w.Pitch_v;
    return b;
}

void listen2car()
{
    while(1)
    {
        unsigned char data[10] = {0xDA,
                               0x00,0x00,  // Yaw_v
                               0x00,0x00,
                               0x00,0x00,  //Pitch_v
                               0x00,0x00,
                               0xDB};
        serial.ReadData(data, 10);
        // 这里可能需要一个标志位,告诉我旋转...
        if (data[0] == 0xDA && data[9] == 0xDB)
        {
            ptz_w.Yaw_v  = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | (data[4]);
            ptz_w.Pitch_v = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | (data[8]);

        }

#ifdef show_serial_listen
       // std::cout << std::hex <<(int)data[1]<<std::endl;//<<(int)data[2]<<(int)data[3]<<(int)data[4]<<std::endl;
        std::cout  << "陀螺仪\tYaw_V: "<<  ptz_w.Yaw_v /1000.0 << "\tPitch_V: "<< ptz_w.Pitch_v/1000.0 << std::endl;
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //return ptz_w;
    }
}

void publish2car(const vision_mul::armor_pos& pos, short _yaw, short _pitch)
{
    unsigned char send_bytes[] = { 0xFF,        // 头
                                   0x00,        // BOOL
                                   0x00,0x00,   // Yaw
                                   0x00,0x00,   // Pitch
                                   0x00,        // 距离
                                   0xFE};       // 尾
    send_bytes[1] = pos.Flag; //debug    // 标志位
    send_bytes[6] = pos.angle_z;  // 距离信息 
//    
    short* data_ptr = (short *)(send_bytes + 2); // 16位指针指向第一个数据
//
    
    data_ptr[0] =/* _yaw   - */static_cast<short>(pos.angle_x * 100);
    data_ptr[1] =/* _pitch - */static_cast<short>(pos.angle_y * 100);  //这边结算不知道为啥反了 Picth轴
    
    //2018.12.23 debug
    //data_ptr[0] = 0 - ptz_w.Yaw_v /1000.0;
    //data_ptr[1] = 0 - ptz_w.Pitch_v /1000.0;
#ifdef show_serial_publish
    
    std::cout << "send_data...\t" 
//              << std::hex 
//              << (int)send_bytes[0] <<"\t\t"
//              <<(int)send_bytes[1]<<"\t\t"
//              <<(int)send_bytes[2]<<"\t\t"
//              <<(int)send_bytes[3]<<"\t\t"
//              <<(int)send_bytes[4]<<"\t\t"
//              <<(int)send_bytes[5]<<"\t\t"
//              <<(int)send_bytes[6]<<"\t\t"
//              <<(int)send_bytes[7]<<"\t\t"<<std::endl;
              << pos.Flag  << "\t\t"
              << _yaw/100.0 <<" - " << pos.angle_x << "\t\t"
              << _pitch/100.0 <<" - " << pos.angle_y << "\t\t"
              << pos.angle_z << std::endl;
#endif

    serial.WriteData(send_bytes, 8);
}

} // namespace serial_mul
} // namespace autocar
