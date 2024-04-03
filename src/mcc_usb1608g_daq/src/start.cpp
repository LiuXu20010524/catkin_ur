#include "mcc_usb1608g.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "unistd.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_mcc_1608g");

  int nread = 8;
  int nchannel = 4;
  double frequency = 1000.0;
  bool isContinuouslyScan = false;

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::Float64MultiArray>("daq", 1000);

  ros::Rate collect_rate(frequency / nread);

  auto daq = MCC_USB1608G::getInstance();
  daq->setTimeout(1000);
  daq->setNRead(nread);
  daq->setNChannel(nchannel);
  daq->setContinuousScanMode(isContinuouslyScan);
  daq->setMode(SINGLE_ENDED);
  daq->setGain(BP_10V);
  daq->setFrequency(frequency);
  if (daq->config() != MCC1608G_SUCCESS)
    return 1;
  daq->start();

  std_msgs::Float64MultiArray msg;

  msg.layout.data_offset = 0;
  msg.layout.dim.resize(2);
  msg.layout.dim[0].size = nread;
  msg.layout.dim[0].label = "Time";
  msg.layout.dim[0].stride = nchannel;
  msg.layout.dim[1].size = nchannel;
  msg.layout.dim[1].label = "Channel";
  msg.layout.dim[1].stride = 1;

  msg.data.resize(nread * nchannel);

  int cnt = 0;
  while (ros::ok()) {
    if (!(cnt & 0xf))
      printf("\n0x%08X ", cnt);
    auto data = new double[nread * nchannel];
    auto ret = daq->read(data);
    cnt++;
    printf(".");

    for (auto i = 0; i < nread * nchannel; i++) {
      msg.data[i] = data[i];
    }
    pub.publish(msg);
  }
  daq->stop();
}
