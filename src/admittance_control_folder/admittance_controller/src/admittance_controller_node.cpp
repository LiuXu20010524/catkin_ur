#include "admittance_controller/AdmittanceController.h"

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"lh_admittance_controller_node");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    
    ros::AsyncSpinner spinner(6);
    spinner.start();

    AdmittanceController controller(nh);

    controller.printCurrentFrame();
    controller.printCurrentJoint();

    printf("Start admittance control ? [y/n]? \t");
    char ch;
    while(std::cin >> ch ){
        ch = tolower(ch);
        if(ch == 'y')
            break;
        else if(ch == 'n')
            return 0;
        else{
            printf("Press error key! \n");
            printf("Start admittance control ? [y/n]? \t");
            continue;
        }
    }
    std::cin.get();

    controller.run();
    return 0;
}
