//
// Created by dell on 2020/12/25.
//

#include "admittance_controller/onlineFilter.h"
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <Eigen/Core>

int main(int argc, char** argv){
    ros::init(argc, argv, "filter_test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();

    onlineFilter f(4, 10, 125, 3);

    std::ifstream dataFile("/home/dell/Nutstore_Files/data_analysis/data/data.txt");        // DataLink记录的一次钻孔力数据
    FILE* outfile = nullptr;
    outfile = fopen("/home/dell/Nutstore_Files/data_analysis/data/filter_result.txt", "w");
    double x1, x2, x3;
    MatrixXd raw(1,3), data(1,3);

    while(dataFile >> x1 >> x2 >> x3) {
        raw(0,0) = x1;
        raw(0,1) = x2;
        raw(0,2) = x3;
        f.filter(raw, data);
        cout << "y = " << data << endl;
        fprintf(outfile, "%f\t %f\t %f\n", data(0,0), data(0,1), data(0,2));
    }
    return 0;

}