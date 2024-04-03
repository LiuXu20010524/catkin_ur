#include "mcc_usb1608g.h"
#include "iostream"
#include "fstream"
MCC_USB1608G *MCC_USB1608G::instance_ = nullptr;

MCC_USB1608G::MCC_USB1608G()
        : is_configured_(false), frequency_(1000), gain_(BP_10V), mode_(DIFFERENTIAL),
          nchan_(16), timeout_(1000), nread_(1), sdataIn_(nullptr)
{
    nh_ = new ros::NodeHandle();
    msg_.header.frame_id = "ATI_force_sensor";

    butterWorth_ = new onlineFilter(4, 15 , 1000, 6);    // 4阶10Hz采样频率1000Hz ButterWorth滤波器，滤波数据为1×6维

    wrench_raw_.resize(1,6);
    wrench_filtered_.resize(1,6);
    wrench_raw_=Eigen::MatrixXd::Zero(1,6);
    wrench_filtered_=Eigen::MatrixXd::Zero(1,6);


//FT33268
    calib_mat_ << -1.93981, 0.33328, 3.62966, -48.09297, -0.18993, 46.65288,
    -1.15609, 54.27486, 0.43722, -27.76496, 1.32341, -26.76951,
    67.12252, 0.35481, 67.46998, 1.08218, 66.84391, 0.73176,
    -0.01306, 0.38587, -1.07298, -0.22046, 1.08918, -0.17263,
    1.26828, 0.00471, -0.66551, 0.32910, -0.62219, -0.33565,
    0.02487, -0.69956, 0.01235, -0.69579, 0.02896, -0.68482;
}

//FT16738
//calib_mat_ << 1.50345, 0.35832, 9.22847, -95.54497 ,-9.83021, 93.35236,
//-17.12073, 105.55710, 7.89457 ,-54.54626 ,5.99915 ,-53.04281,
//135.12422, 10.70435, 133.23491, 9.04286 ,134.87261, 5.68562,
//-0.09684 ,0.73409, -2.10309, -0.52814 ,2.21791 ,-0.27266,
//2.48870, 0.18899 ,-1.28885 ,0.57327 ,-1.21080 ,-0.69835,
//0.16480 ,-1.38482, 0.19946, -1.40424, 0.16990 ,-1.37691;

//FT33268
//calib_mat_ << -1.93981, 0.33328, 3.62966, -48.09297, -0.18993, 46.65288,
//-1.15609, 54.27486, 0.43722, -27.76496, 1.32341, -26.76951,
//67.12252, 0.35481, 67.46998, 1.08218, 66.84391, 0.73176,
//-0.01306, 0.38587, -1.07298, -0.22046, 1.08918, -0.17263,
//1.26828, 0.00471, -0.66551, 0.32910, -0.62219, -0.33565,
//0.02487, -0.69956, 0.01235, -0.69579, 0.02896, -0.68482;

//calib_mat_ 2022.11.2

//calib_mat_ << -0.13120, 0.22829, 1.78002, -47.67547, -8.86098, 45.49954,
//-3.64575, 54.81379, -0.88146, -27.50522, 6.54723, -26.43828,
//67.89496, 3.64485, 70.04207, 3.46360, 63.65626, 4.10091,
//-0.04042, 0.39110, -1.08082, -0.25500, 1.10200, -0.11567,
//1.28160, 0.07040, -0.65149, 0.30709, -0.54974, -0.36358,
//0.05402, -0.69882, 0.04110, -0.69597, 0.09768, -0.68012;

MCC_USB1608G::MCC_USB1608G(MCC_USB1608G &)
{}

MCC_USB1608G::~MCC_USB1608G()
{
  this->stop();
  delete[] sdataIn_;
}

MCC_USB1608G *MCC_USB1608G::getInstance()
{
  if (MCC_USB1608G::instance_ == nullptr)
    MCC_USB1608G::instance_ = new MCC_USB1608G();
  return MCC_USB1608G::instance_;
}

void MCC_USB1608G::publishToTopic(bool isPrint){
    daq_pub_ = nh_->advertise<geometry_msgs::WrenchStamped>("mcc_1608g_daq", 100);      // TCP通信缓冲队列大小:100
    loop_rate_ = new ros::Rate(frequency_ / nread_);
    // config MCC1608G
    if (config() != MCC1608G_SUCCESS){
        ROS_ERROR("Fail to config MCC_1608G! Please check and try again ...");
        return;
    }

    // start MCC1608G
    start();

    // acquire the bias
    biasMeasure();
    ros::Time start = ros::Time::now();
    ros::Duration du;
    // read and publish the data
    msg_.header.frame_id = "ATI_sensor";
    while (ros::ok()) {
        auto data = new double[nread_ * nchan_];
        auto ret = read(data);

        Eigen::VectorXd v(6);
        for(int i=0; i<6; i++){
            v(i) = data[(i + 1) * nread_ - 1] - forceBias_[i];
//            v(i) = data[(i + 1) * nread_ - 1] ;
        }

        v = calib_mat_ * v;

        msg_.header.stamp = ros::Time::now();

      //  msg_.wrench.force.x = v(0);
      //  msg_.wrench.force.y = v(1);
      //  msg_.wrench.force.z = v(2);
      //  msg_.wrench.torque.x = v(3);
      //  msg_.wrench.torque.y = v(4);
      //  msg_.wrench.torque.z = v(5);

        wrench_raw_(0,0)=v(0);
        wrench_raw_(0,1)=v(1);
        wrench_raw_(0,2)=v(2);
        wrench_raw_(0,3)=v(3);
        wrench_raw_(0,4)=v(4);
        wrench_raw_(0,5)=v(5);

        butterWorth_->filter(wrench_raw_, wrench_filtered_);      // 对ATI力传感器的数据进行滤波

        msg_.wrench.force.x = wrench_filtered_(0,0);
        msg_.wrench.force.y = wrench_filtered_(0,1);
        msg_.wrench.force.z = wrench_filtered_(0,2);
        msg_.wrench.torque.x = wrench_filtered_(0,3);
        msg_.wrench.torque.y = wrench_filtered_(0,4);
        msg_.wrench.torque.z = wrench_filtered_(0,5);

        if(isPrint){
            ROS_INFO("The %d daq sample:", nread_);
            for(int i=0; i<nchan_; i++){
                std::cout << "[ ";
                for(int j=0; j<nread_; j++){
                    std::cout << data[i*nread_ + j] << " ";
                }
                std::cout << "]" << std::endl;
            }
        }

        daq_pub_.publish(msg_);             // publish the msg
        du=msg_.header.stamp-start;
        loop_rate_->sleep();
    }
}

void MCC_USB1608G::biasMeasure()
{
    double baseTemp[6];
    int readTimes = 5000;

    // 先将bias里面的值清零
    for(int i = 0; i < 6; i++)
        forceBias_[i] = 0;

    std::cout << "reading bias, please wait..." << std::endl;
    for(int i = 0; i < readTimes; i++)
    {
        read(baseTemp);
        for(int j = 0; j < 6; j++)
            forceBias_[j] += baseTemp[j];

        usleep(500);
    }

    for(int j = 0; j < 6; j++)
        forceBias_[j] = forceBias_[j] / (double) readTimes;

    std::cout << "Bias of ATI sensor has acquired!" << std::endl;
}

int MCC_USB1608G::config()
{
    if (is_configured_)
    this->stop();

    // 一开始的做法是把零点漂移值保存在文件中，每次启动的时候读取文件
    // 后来干脆直接每次实验的时候都重新采集零点漂移值，就把下面都注释了
    // 读取力初始值，如果读取文件失败，则将所有值置为零
//    FILE *biasfile;
//    biasfile = fopen("/home/dell/UR_Control/ur_catkinws/src/mcc_usb1608g_daq/src/calibrationfiles/forcebias.txt", "r");
//    if(biasfile == NULL)
//    {
//        forceBias_[0] = 0;
//        forceBias_[1] = 0;
//        forceBias_[2] = 0;
//        forceBias_[3] = 0;
//        forceBias_[4] = 0;
//        forceBias_[5] = 0;
//        ROS_WARN( "NO calibration file exist in /home/dell/UR_Control/ur_catkinws/src/mcc_usb1608g_daq/src/calibrationfiles \n" );
//        printf(" forceBias = [%f, %f, %f, %f, %f, %f] \n",
//                   forceBias_[0], forceBias_[1], forceBias_[2],
//                   forceBias_[3], forceBias_[4], forceBias_[5] );
//    }
//    else
//    {
//        fscanf(biasfile, "%lf, %lf, %lf, %lf, %lf, %lf",
//                           &forceBias_[0], &forceBias_[1], &forceBias_[2],
//                           &forceBias_[3], &forceBias_[4], &forceBias_[5]);
//        printf("forceBias has read from file: /home/geds/ros_space/robot_ws/src/ati_force_sensor/src/calibrationfiles/forcebias.txt \n");
//        printf(" forceBias = [%f, %f, %f, %f, %f, %f] \n",
//                   forceBias_[0], forceBias_[1], forceBias_[2],
//                   forceBias_[3], forceBias_[4], forceBias_[5] );
//        fclose(biasfile);
//    }

    udev_ = nullptr;
    is_configured_ = false;

    // initialize libusb
    auto ret = libusb_init(nullptr);
    if (ret < 0)
    return MCC1608G_NOTFOUND;

    // find USB1608G
    if ((udev_ = usb_device_find_USB_MCC(USB1608G_PID, nullptr))) // version 1
    usbInit_1608G(udev_, 1);
    else if ((udev_ = usb_device_find_USB_MCC(USB1608G_V2_PID, nullptr))) // version 2
    usbInit_1608G(udev_, 2);
    else
    return MCC1608G_NOTFOUND;

    printf("USB1608G found @0x%lX\n", (long) udev_);

    // select mode
    wMaxPacketSize_ = usb_get_max_packet_size(udev_, 0);

    if (is_continuous_mode_) {
    if ((nchan_ * nread_ * 2 < wMaxPacketSize_) ||
        (nchan_ * nread_ * 2 % wMaxPacketSize_ != 0))
      return MCC1608G_CONFIGFAILED;
    }
    else {
    if (nchan_ * nread_ * 2 > wMaxPacketSize_)
      return MCC1608G_CONFIGFAILED;
    }

    usbBuildGainTable_USB1608G(udev_, table_AIN_);

    // configure hardware
    // write channel settings
    ScanList list[16];
    for (uint8_t i = 0; i < nchan_; i++) {
    list[i].range = gain_;
    list[i].channel = i;
    list[i].mode = mode_;
    }
    list[nchan_ - 1].mode |= LAST_CHANNEL;
    // printf("nchan_ = %d \n",nchan_);
    // for (int i = 0;i<nchan_;i++)
    //   printf("%d channel = %d \n",i,list[i].channel);

    usbAInScanStop_USB1608G(udev_);
    usbAInConfig_USB1608G(udev_, list);
    sleep(1);

    delete[] sdataIn_;
    sdataIn_ = new uint16_t[nchan_ * nread_];

    is_configured_ = true;
    return MCC1608G_SUCCESS;
}

void MCC_USB1608G::setContinuousScanMode(bool isContinuouslyScan)
{
  is_continuous_mode_ = isContinuouslyScan;
}

int MCC_USB1608G::start()
{
  if (!is_configured_)
    return MCC1608G_NOTALLOWED;

  uint8_t nscans = (is_continuous_mode_) ? 0 : nread_;
  usbAInScanStart_USB1608G(udev_, nscans, 0, frequency_, 1);
  return MCC1608G_SUCCESS;
}

int MCC_USB1608G::stop()
{
  if (!is_configured_)
    return MCC1608G_NOTALLOWED;
  usbAInScanStop_USB1608G(udev_);
  usbAInScanClearFIFO_USB1608G(udev_);
  return MCC1608G_SUCCESS;
}

int MCC_USB1608G::read(double *data)
{
  auto len = this->readRawData(sdataIn_);
  for (auto i = 0; i < nread_; i++)
    for (auto j = 0; j < nchan_; j++) {
      auto k = i * nchan_ + j;
      uint16_t temp = rint(sdataIn_[k] * table_AIN_[gain_][0] + table_AIN_[gain_][1]);
      data[k] = volts_USB1608G(gain_, temp);
    }
  return len;
}

int MCC_USB1608G::readRawData(uint16_t *sdataIn)
{
  if (!is_configured_)
    return MCC1608G_CONFIGFAILED;
  if (!is_continuous_mode_)
    this->start();
  return usbAInScanRead_USB1608G(udev_, nread_, nchan_, sdataIn, timeout_, CONTINUOUS) >> 1;
}

void MCC_USB1608G::setFrequency(double frequency)
{ frequency_ = frequency; }

void MCC_USB1608G::setGain(uint8_t gain)
{
  switch (gain) {
    case BP_10V:
    case BP_5V:
    case BP_2V:
    case BP_1V:
      gain_ = gain;
      break;
    default:
      gain_ = BP_10V;
  }
}

void MCC_USB1608G::setMode(uint8_t mode)
{
  if ((mode == SINGLE_ENDED) || (mode == DIFFERENTIAL))
    mode_ = mode;
  else mode_ = DIFFERENTIAL;
}

void MCC_USB1608G::setNChannel(int nchan)
{ nchan_ = nchan; }

void MCC_USB1608G::setNRead(int nread)
{ nread_ = nread; }

void MCC_USB1608G::setTimeout(unsigned int timeout)
{ timeout_ = timeout; }

void MCC_USB1608G::blink(uint8_t count)
{
  if (udev_ != nullptr)
    usbBlink_USB1608G(udev_, count);
}
