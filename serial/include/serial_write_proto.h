/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _SERIAL_WRITE_H_
#define _SERIAL_WRITE_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>

#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/Twist.h>


using namespace std;
using namespace boost::asio;




struct Serial2CarProto
{
    float yaw_angle;
    float pitch_angle;
    // float distance;
    // int   mode;
    // bool  shoot;
}s2c;
namespace serial_mul
{

struct buffer_write
{
    uint8_t sof;
    // int16_t auto_vx;     // advacne control speed
    // int16_t auto_vy;     // translation control speed
    // int16_t auto_vw;     // steering control speed

    int16_t yaw;
    int16_t pitch;

    uint8_t end;

}/*__attribute__((packed)) */data_t;

class serial_write
{   
    io_service m_ios;				// io_service Object
	serial_port *pSerialPort;		// Serial port Object
	std::string port_id;	    	// For save com name
	boost::system::error_code ec;	// Serial_port function exception

public:
    serial_write(): port_id("/dev/ttyUSB0"),
                    enemy_detected_(false),
                    lost_cnt_(0),
                    detect_cnt_(0)
                    // armor_detection_actionlib_client_("armor_detection_node_action", true)
    {
        pSerialPort = new serial_port(m_ios);

        if (pSerialPort){
            ros::param::get("serial_port",port_id);
			if (init_port( port_id, 8 ))
                cout << "init serial [ " << port_id << " ] success! write data... \n";
		}
    }

    ~serial_write() { if(pSerialPort) delete pSerialPort; }

public:
    void write_To_serial(struct Serial2CarProto s2c)  //const detect::armor_goal& vision_data
    {
        ROS_ERROR("pitch: %f , yaw: %f", s2c.pitch_angle, s2c.yaw_angle);
        uint8_t data[6];
        data[0] = 0xff;
        data[1] = uint8_t(int16_t(s2c.pitch_angle) ) ;
        data[2] = uint8_t(int16_t(s2c.pitch_angle) >> 8 ); 
        data[3] = uint8_t(int16_t(s2c.yaw_angle) ) ;
        data[4] = uint8_t(int16_t(s2c.yaw_angle) >> 8);
        data[5] = 0xfe;

        size_t len = write( *pSerialPort, buffer( data ), ec );
        // cout << "send length: " << len << "\tbuf: " << data << "\n";
        // ROS_ERROR("%d", data[0]);
        // ROS_ERROR("%d", data[1]);
        // ROS_ERROR("%d", data[2]);
        // ROS_ERROR("%d", data[3]);
        // ROS_ERROR("%d", data[4]);
        // ROS_ERROR("%d", data[5]);
        // ROS_ERROR("===============================");
        
    }

    
private:


    bool enemy_detected_;
    int lost_cnt_;   
    int detect_cnt_;   

	bool init_port( const std::string port, const unsigned int char_size = 8)
    {
        if (!pSerialPort) return false;

        pSerialPort->open( port, ec );
        
        pSerialPort->set_option( serial_port::baud_rate( 115200 ), ec );
        pSerialPort->set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );
        pSerialPort->set_option( serial_port::parity( serial_port::parity::none ), ec );
        pSerialPort->set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);
        pSerialPort->set_option( serial_port::character_size( char_size ), ec);
    
        return true;
    }
};

    

} // namespace serial_mul


#endif
