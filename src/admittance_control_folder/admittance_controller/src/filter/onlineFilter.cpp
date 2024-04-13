//
// Created by dell on 2020/12/25.
//

#include "admittance_controller/onlineFilter.h"

onlineFilter::onlineFilter(unsigned int order, double fc, double freq, unsigned int x_width){
    this->order_ = order;
    this->fc_ = fc;
    this->freq_ = freq;
    this->x_width_ = x_width;

    // 计算Butter滤波器系数
    b_.resize(1, order_+1);     b_ = MatrixXd::Zero(1, order+1);
    a_.resize(1, order_+1);     a_ = MatrixXd::Zero(1, order+1);

    TransferFunction<double> lowPassButter(butter<double>(order_, fc_, freq_));

    for(int i=0; i<=order; i++){
        b_(0, i) = lowPassButter.b(i).real();
        a_(0, i) = lowPassButter.a(i).real();
    }

    x_old_.resize(order_, x_width_);          x_old_ = MatrixXd::Zero(order_, x_width_);
    y_old_.resize(order_, x_width_);          y_old_ = MatrixXd::Zero(order_, x_width_);

    x_temp_.resize(order_+1, x_width_); x_temp_ = MatrixXd::Zero(order_+1, x_width_);
    y_temp_.resize(order_+1, x_width_); y_temp_ = MatrixXd::Zero(order_+1, x_width_);

    printf("IIR Filter, %d_order Low Pass Butterworth, cutoff frequency %fHz, sample frequency %fHz, x_width: %d\n", order_, fc_, freq_, x_width_);

}

void onlineFilter::filter(const MatrixXd& x, MatrixXd& y) {

//     x_temp = [x_old; x];
    x_temp_.topRows(order_) = x_old_;
    x_temp_.bottomRows(1) = x;

    // y_temp = [y_old; zeros(size(x))];
    y_temp_.topRows(order_) = y_old_;
    y_temp_.bottomRows(1) = MatrixXd::Zero(1, x_width_);

//    for i = obj.order+1 : obj.order+ x_length
//      y_temp(i,:) = sum(obj.b* x_temp(i:-1:i-obj.order,:),1) - sum(obj.a(2:end)*y_temp(i-1:-1:i-obj.order,:),1);
//    end
    MatrixXd tmp1 = x_temp_;
    for(int i=0; i<tmp1.rows()/2; i++){
        tmp1.row(i).swap(tmp1.row(tmp1.rows()-i-1));
    }
    MatrixXd sum1 = b_ * tmp1;

    MatrixXd tmp2 = y_temp_.topRows(order_);
    for(int i=0; i<tmp2.rows()/2; i++){
        tmp2.row(i).swap(tmp2.row(tmp2.rows()-i-1));
    }
    MatrixXd sum2 = a_.rightCols(a_.cols()-1) * tmp2;

    y_temp_.row(order_) = sum1 - sum2;

//    y = y_temp(obj.order+1:end,:);
//    obj.x_old = x_temp(end-obj.order+1:end,:);
//    obj.y_old = y_temp(end-obj.order+1:end,:);
    y = y_temp_.bottomRows(1);
    x_old_ = x_temp_.bottomRows(order_);
    y_old_ = y_temp_.bottomRows(order_);
}

void onlineFilter::setFilterOrder(unsigned int order){
    this->order_ = order;
}

void onlineFilter::setFilterFc(double fc){
    this->fc_ = fc;
}

void onlineFilter::setFilterFreq(double freq){
    this->freq_ = freq;
}

void onlineFilter::setFilterXWidth(unsigned int x_width){
    this->x_width_ = x_width;
}