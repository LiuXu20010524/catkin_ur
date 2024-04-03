//
// Created by dell on 2020/11/13.
//

#include "mcc_usb1608g.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_mcc_1608g");

    int nread = 1;
    int nchannel = 6;
    double frequency = 1000.0;
    bool isContinuouslyScan = false;

    auto daq = MCC_USB1608G::getInstance();
    daq->setTimeout(1000);
    daq->setNRead(nread);
    daq->setNChannel(nchannel);
    daq->setContinuousScanMode(isContinuouslyScan);
    daq->setMode(SINGLE_ENDED);
    daq->setGain(BP_10V);
    daq->setFrequency(frequency);

    if (daq->config() != MCC1608G_SUCCESS){
        ROS_ERROR("Fail to config MCC1608G");
        return 1;
    }
    daq->start();

    // acquire the bias
    daq->biasMeasure();

    daq->stop();
}
