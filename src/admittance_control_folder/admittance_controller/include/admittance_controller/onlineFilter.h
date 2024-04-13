//
// Created by dell on 2020/12/25.
//

#ifndef SRC_ONLINEFILTER_H
#define SRC_ONLINEFILTER_H

#include "rtb/Filter/Polynomial.h"
#include "rtb/Filter/Filter.h"
#include "rtb/Filter/TransferFunction.h"
#include "rtb/Filter/Designer.h"
#include "rtb/Filter/StateSpaceFilter.h"
#include "Eigen/Eigen"

using namespace rtb::Filter;
using namespace Eigen;
using namespace std;

class onlineFilter{
private:
    unsigned int order_;
    double fc_;
    double freq_;
    unsigned int x_width_;
    MatrixXd b_, a_;
    MatrixXd x_old_, y_old_;
    MatrixXd x_temp_, y_temp_;
public:
    /**
     * @brief 在线形式ButterWorth 低通滤波器，输入数据为1×N维
     * @param order: 滤波器阶数
     * @param fc: 截止频率
     * @param freq: 采样频率
     * @param x_width: 数据维度
     */
    onlineFilter(unsigned int order, double fc, double freq, unsigned int x_width);

    /**
     * @brief 滤波函数
     * @param x: 原始数据(1×N维）.如果想要M×N维的滤波器，自己去改吧，我懒得写了
     * @param y: 滤波后数据(1×N维）
     */
    void filter(const MatrixXd& x, MatrixXd& y);

    void setFilterOrder(unsigned int order);

    void setFilterFc(double fc);

    void setFilterFreq(double freq);

    void setFilterXWidth(unsigned int x_width);

};
#endif //SRC_ONLINEFILTER_H
