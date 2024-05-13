# README

This ROS workspace was created by Xu Liu.

This ws contains the following packages:

- src/admittance_control_folder/**admittance_controller**: this package was created by lh, using twist_controller to implement admittance control of UR5 robotic arm
- src/admittance_control_folder/**shear_thickening_fluid_controller**: this package was created by Xu Liu, is a reproduction of the paper: "Compliance while resisting: A shear-thickening fluid controller for physical human-robot interaction". The DOI is : 10.1177/02783649241234364
- src/**mcc_usb1608g_daq**: this package is a package that uses ATI Mini45 sensors. It filters the data by default and publishes sensor data topics.
- src/trajopt/**traj_phri_deformation**: this package was created by Xu Liu, is a reproduction of the paper: "Trajectory Deformations From Physical Human–Robot Interaction". The DOI is : 10.1109/TRO.2017.2765335

The remaining folders or packages are either packages used for testing or packages related to the UR robot arm that have been open sourced on github.

All the codes(In addition to open source libraries and codes related to the UR robotic arm) in this workspace are basically written by the author or the author's collaborators. Therefore, the code may have many loopholes and has not been optimized and perfected. If anyone needs to use it, please pay attention to the safety of the equipment. Colleagues are also welcome to give criticism and suggestions.