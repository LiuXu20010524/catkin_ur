ROS driver for MCC-USB1608G DAQ
======================

Only support analog input.

Message Type
--------

`std_msgs/Float64MultiArray`

The msg.Data are aligned as

`u(0,0), u(1,0), u(2,0), ..., u(0,1), u(1,1), u(2,1), ..., u(k,t)`

where `k` means k-th channel and `t` means t-th sample.

