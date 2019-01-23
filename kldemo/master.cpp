#include "kldemo.hpp"

int main(int argc, char const *argv[])
{
    VideoCapture cap;
    cap.open(0);
    Mat frame;

    while(1){
        cap>>frame;
        kldemo(frame);
        
    }
    return 0;
}
