#ifndef ADDMITTANCE_CONTROLLER_JOINT_TRAJ_H
#define ADDMITTANCE_CONTROLLER_JOINT_TRAJ_H

// --- Include --- //
#include <pinocchio/parsers/urdf.hpp> //Note:The pinocchio declaration needs to precede the ros declaration.
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <cmath>

#define PI 3.14159265358979323846
namespace pin = pinocchio;

class addmittance_controller_joint_traj
{
public:
// --- Variables --- //

// --- Functions --- //
    addmittance_controller_joint_traj();
    ~addmittance_controller_joint_traj();
};



#endif