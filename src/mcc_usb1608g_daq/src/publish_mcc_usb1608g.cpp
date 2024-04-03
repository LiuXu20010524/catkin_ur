#include "mcc_usb1608g.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_mcc_1608g");
//    ros::AsyncSpinner spinner(5);
//    spinner.start();
    auto daq = MCC_USB1608G::getInstance();
    daq->setTimeout(1000);
    daq->setNRead(1);
    daq->setFrequency(1000);
    daq->setNChannel(6);
    daq->setContinuousScanMode(false);
    daq->setMode(SINGLE_ENDED);
    daq->setGain(BP_10V);

    daq->publishToTopic(false);

    return 0;
}
