#include "serial_device.h"

void serialCallback(const pass)
{
    pass
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle serial_nh_;
    ros::Publisher  pub;
    ros::Subscriber sub;
    roborts_sdk::SerialDevice serial(/dev/ttyUSB0,115200); //just test
    ros::Rate sleep_rate(10);

    pub = serial_nh_.advertise<pass>("pass",100);
    sub = serial_nh_.subscribe("pass", 100, serialCallback);
//shuang xian cheng?
    while(ros::ok){
        serial.Read(pass);
        serial.Write(pass);


        sleep_rate.sleep();
    }
    return 0;
}