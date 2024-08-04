#include "dwa.h"
#include "vel_prof.h"
#include <ros/console.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PP_DwaPlanner");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    dwaLocalPlanner dwa_object = dwaLocalPlanner();

    bool newPath = false;
    std::string statess;
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::param::get("/PP/new_path",newPath);
        ros::param::get("/state", statess);

        if(statess != "ManualControl"){
            if (newPath){
                // std::cout<<"new path param set : "<<std::endl;
                dwa_object.kdt.clear();
                dwa_object.half_path = false;
                dwa_object.first_half_path.clear();
                dwa_object.second_half_path.clear();
                dwa_object.KdTreeGenerated = false;
                dwa_object.globalPath.clear();
                dwa_object.pathReceived = false;
                dwa_object.corien_pathReceived = false;
                dwa_object.recovery_pathReceived = false;
                ros::param::set("/PP/new_path",false);
                newPath = false;
                // ros::param::set("/PP/end_of_trajectory",false);
                dwa_object.localPath.clear();
                // std::cout<<"cleared everything"<<std::endl;
            }

            if (dwa_object.startposereceived && (dwa_object.pathReceived || dwa_object.corien_pathReceived || dwa_object.recovery_pathReceived)) {
                // std::cout<<"going in driver"<<std::endl;
                dwa_object.driver();
                ros::param::set("/PP/path_recieved", true);
            }
            // if(dwa_object.pathReceived || dwa_object.corien_pathReceived || dwa_object.recovery_pathReceived){
            //     ros::param::set("/PP/path_recieved", true);
            // }
        }
        else{
            ros::Rate(1).sleep();
            // std::cout<<"###################"<<std::endl;
            // std::cout<<"idling"<<std::endl;
             dwa_object.end_of_traj = true;
             dwa_object.globalPath.clear();
             dwa_object.kdt.clear();
             dwa_object.first_half_path.clear();
             dwa_object.second_half_path.clear();
             dwa_object.KdTreeGenerated = false;
             dwa_object.pathReceived = false;
             dwa_object.recovery_pathReceived = false;
             dwa_object.corien_pathReceived = false;
            //  dwa_object. msg.data.clear();
            //  dwa_object. msg.data.push_back(0);
            //  dwa_object. vel_publisher.publish(msg);
        }
        ros::spinOnce();

        rate.sleep();
    }
       
    return 0;
}
