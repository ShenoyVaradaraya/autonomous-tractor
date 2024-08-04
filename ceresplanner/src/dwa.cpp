#include "dwa.h"
#include "vel_prof.h"
#include "globalVelocity.h"
#define RAD2DEG(x) ((x)*180./M_PI)

gen_vel vp_dwa;


dwaLocalPlanner::dwaLocalPlanner()
{
    ros::NodeHandle n; 
    ros::NodeHandle n_private("~");
    pi = M_PI;
    MAX_STEERING_ANGLE = 1.39;//pi/2.2;  //# TP
    steeringAngle = 90;
    assignSteering();
    VELOCITY = 1.5;  //# TP
    TIME =              1.5;  //# TP
    DT =                0.075;
    TRACK_POINT =       8;  //# TP dwa trajectory
    PATH_POINT =        8; // local path trajectory
    NUM_DIGIT =         20;  //# 12, 15 #TP
    heading_threshold = 0.999;
    counter = 0;
    MAX_ACC = 0.5;
    prevAccln = 0;
    velocity_resolution = 0.05;
    currVelocity = 1.0;
    GOAL_COST_GAIN =  1.0;
    SPEED_COST_GAIN = 0.0;
    OBS_COST_GAIN =  0;
    LENGTH = 2.3;
    LR = 1.2;
    GOAL_THRESHOLD =      2;
    startposereceived =   false;
    localgoalreceived =   false;
    pathReceived =        false;
    targetSpeedReceived = false;
    recovery_pathReceived = false;
    corien_pathReceived = false;
    track_point_recieved = false;


    KdTreeGenerated = false;
    half_path_point = -1;
    half_path = false;
    velocity_factor = 0.8;
    goalRegionThreshold = 0.5;
    feedbackVelocity = 0;
   
    path_publisher       = n.advertise<nav_msgs::Path>("/PP/dwa_localpath", 10); // what is the msg type for the path msg??????
    localpath_publisher  = n.advertise<nav_msgs::Path>("/PP/LocalPath", 10); // what is the msg type for the path msg??????
    vel_publisher        = n.advertise<std_msgs::Float32MultiArray>("/PP/Speed", 10); 
    pathLengthPub        = n.advertise<std_msgs::Float32>("/PP/PathLength", 10); 
    sub_ndt_pose         = n.subscribe("/ndt_pose", 1,    &dwaLocalPlanner::getVehicleNDTPose,        this);
    sub_rviz_pose        = n.subscribe("/initialpose", 1, &dwaLocalPlanner::getVehicleRVIZPose,       this);
    sub_corien_path      = n.subscribe("/PP/CorienPath", 1, &dwaLocalPlanner::callbackcorienpath,     this);
    sub_recovery_path    = n.subscribe("/PP/RecoveryPath", 1, &dwaLocalPlanner::callbackrecoverypath,     this);
    sub_obs_points       = n.subscribe("/obstacle_pose_array",  1, &dwaLocalPlanner::CallBackObstacle,this);
    sub_odin_trackPoints = n.subscribe("tracking_points", 1, &dwaLocalPlanner::CallBackOdin,    this);// add flux_msgs
    sub_global_path      = n.subscribe("/PP/GlobalPath",  1, &dwaLocalPlanner::getGlobal,       this);
    sub_es_inputs = n.subscribe("/ES_inputs", 1, &dwaLocalPlanner::callBackESInputs, this);
    sub_lidar_data = n.subscribe("/2d_lidar/lidar_center/scan", 1000, &dwaLocalPlanner::callBackLidarData, this);

}

void dwaLocalPlanner::callBackLidarData(const sensor_msgs::LaserScan::ConstPtr& scan){
    // int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    // ROS_INFO("angle_increment, %f", RAD2DEG(scan->angle_increment));
    // ROS_INFO("scan_time, %f", (scan->scan_time));
    // ROS_INFO("range_min, %f", (scan->range_min));
    // ROS_INFO("range_max, %f", (scan->range_max));
    // ROS_INFO("time_increment, %f", (scan->time_increment));
    scanMsg.scan_time = scan->scan_time;
    scanMsg.ranges = scan->ranges;
    scanMsg.angle_increment = scan->angle_increment;
    scanMsg.angle_min = scan->angle_min;
    scanMsg.angle_max = scan->angle_max;
    scanMsg.time_increment = scan->time_increment;
}
void dwaLocalPlanner::setTargetCB(const std_msgs::Float32& msg)
{
    targetSpeed = msg.data;
    targetSpeedReceived = true;
}

void dwaLocalPlanner::assignSteering()
{
    for(double iter = -MAX_STEERING_ANGLE; iter < MAX_STEERING_ANGLE; iter+=0.01)
    {
        steering_angle.push_back(iter);
    }
}

void dwaLocalPlanner::callBackESInputs(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // feedbackVelocity = msg->data[21];
    steeringAngle = msg->data[15]; 
    // ROS_INFO("Steering angle is : %f", steeringAngle);
}
void dwaLocalPlanner::getGlobal(const nav_msgs::Path::ConstPtr& msg)
{
   if (!pathReceived)  
    {
        for (int i = 0; i < msg->poses.size(); i++)  
        {
            geometry_msgs::PoseStamped pt = msg->poses[i];
            tf::Pose tfPose;
            tf::poseMsgToTF(pt.pose, tfPose);
            double yaw = tf::getYaw(tfPose.getRotation());
            Node st(pt.pose.position.x, pt.pose.position.y, yaw);
             if(pt.header.seq == 1){
                half_path_point = i;
                half_path = true;
            }
            globalPath.push_back(st);
        }
        pathReceived = true;
        ros::param::get("/PP/direction", current_direction);
        KdTreeGenerated = false;
        ros::param::set("PP/path_done", false);
        esPose.parent = -1;
        ROS_INFO("DWA_planner/plan : global path recieved");
    }
}

void dwaLocalPlanner::callbackcorienpath(const nav_msgs::Path::ConstPtr& msg){
    if (!corien_pathReceived) {
        for (int i = 0; i < msg->poses.size(); i++) {
            geometry_msgs::PoseStamped pt = msg->poses[i];
            tf::Pose tfPose;
            tf::poseMsgToTF(pt.pose, tfPose);
            double yaw = tf::getYaw(tfPose.getRotation());
            Node st(pt.pose.position.x, pt.pose.position.y, yaw);
            globalPath.push_back(st);
        }
        corien_pathReceived = true;
        ros::param::get("/PP/direction", current_direction);
        esPose.parent = -1;
        ros::param::set("PP/path_done", false);
        ROS_INFO("DWA_planner/plan : global path recieved");
    }
}

void dwaLocalPlanner::callbackrecoverypath(const nav_msgs::Path::ConstPtr& msg){
    if (!recovery_pathReceived) {
        for (int i = 0; i < msg->poses.size(); i++) {
            geometry_msgs::PoseStamped pt = msg->poses[i];
            tf::Pose tfPose;
            tf::poseMsgToTF(pt.pose, tfPose);
            double yaw = tf::getYaw(tfPose.getRotation());
            if(isnan(yaw))
                yaw = 0;
            Node st(pt.pose.position.x, pt.pose.position.y, yaw);
            globalPath.push_back(st);
        }
        recovery_pathReceived = true;
        ros::param::get("/PP/direction", current_direction);
        esPose.parent = -1;
        ros::param::set("PP/path_done", false);
        ROS_INFO("DWA_planner/plan : recovery path recieved");
    }
}

int dwaLocalPlanner::genKDTree(int ind, KDT::KDTree<KDT::MyPoint>* kdt,  std::vector<Node> path)  {
    std::vector<KDT::MyPoint> kdt_pts;
    kdt_pts.push_back(KDT::MyPoint(path[ind].x, path[ind].y));
    int ind_max;    
    for (int i = ind+1; i < path.size(); i++)  {
        kdt_pts.push_back(KDT::MyPoint(path[i].x, path[i].y));
        ind_max = i;
    }
    if(kdt_pts.empty()) {
        ROS_DEBUG_THROTTLE(60,"local_planner/plan : KDTree not generated");
    }
    kdt->build(kdt_pts);
    ROS_DEBUG("local_planner/plan : kdtree generated");
    return ind_max;
}


void dwaLocalPlanner::getVehicleNDTPose(const geometry_msgs::PoseStamped::ConstPtr& curr_pose) {
    current_pose.x = curr_pose->pose.position.x;
    current_pose.y = curr_pose->pose.position.y;
    tf::Pose tfPose;
    tf::poseMsgToTF(curr_pose->pose, tfPose);
    current_pose.orien = tf::getYaw(tfPose.getRotation());
    startposereceived = true;
}

void dwaLocalPlanner::getVehicleRVIZPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {   
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    tf::Pose tfPose;
    tf::poseMsgToTF(msg->pose.pose, tfPose);
    current_pose.orien = tf::getYaw(tfPose.getRotation());
    startposereceived = true;
}

void dwaLocalPlanner::CallBackObstacle(const geometry_msgs::PoseArray::ConstPtr& obs_data) {   
    for (auto &obs_pt: obs_data->poses){
        Node obs;
        obs.x = obs_pt.position.x;
        obs.y = obs_pt.position.y;
        obstacles.push_back(obs);
    }
}

void dwaLocalPlanner::CallBackOdin(const flux_msgs::TrackingPoints::ConstPtr& odin_msg) {
    flux_msgs::TrackingPoints odin_localization_wc, odin_localization_dw, odin_localization_cg;

    wc_global.wheel_center.x = odin_msg->fork_center.x;
    wc_global.wheel_center.y = odin_msg->fork_center.y;
    wc_global.wheel_center.z = 0;
    wc.x = wc_global.wheel_center.x;
    wc.y = wc_global.wheel_center.y;
    wc.orien = 0;

    dw_global.driving_wheel.x = odin_msg->driving_wheel.x;
    dw_global.driving_wheel.y = odin_msg->driving_wheel.y;
    dw_global.driving_wheel.z = 0;
    dw.x = dw_global.driving_wheel.x;
    dw.y = dw_global.driving_wheel.y;
    dw.orien = 0;
    cg_global.geometric_center.x = odin_msg->geometric_center.x;
    cg_global.geometric_center.y = odin_msg->geometric_center.y;
    cg_global.geometric_center.z = 0;
    cg.x = cg_global.geometric_center.x;
    cg.y = cg_global.geometric_center.y;
    cg.orien = 0;
    track_point_recieved = true;

}

bool dwaLocalPlanner::traj_direction_chooser() {
    bool direction_switch;
    if (current_direction != goal_direction)
        direction_switch = true;
    else
        direction_switch = false;

    return direction_switch;
}


long double dwaLocalPlanner::calc_to_goal_cost(std::vector<Node>& traj, Node& goal) {
    std::pair<long double, long double> traj_vec = std::make_pair(traj[traj.size()-1].x - curr_x,
                    traj[traj.size()-1].y - curr_y);
    std::pair<long double, long double> goal_vec = std::make_pair(goal.x - curr_x,
                    goal.y - curr_y);

    long double norm1 = std::pow(std::pow(traj_vec.first, 2) + std::pow(traj_vec.second, 2) , 0.5);
    long double norm2 = std::pow(std::pow(goal_vec.first, 2) + std::pow(goal_vec.second, 2) , 0.5);  

    std::pair<long double , long double> unit_traj_vec = std::make_pair(traj_vec.first/norm1, traj_vec.second/norm1);
    std::pair<long double, long double>  unit_goal_vec = std::make_pair(goal_vec.first/norm2, goal_vec.second/norm2);
    long double heading = unit_traj_vec.first * unit_goal_vec.first + unit_goal_vec.second * unit_traj_vec.second;
    heading = (1 + heading)/2;
    return roundd(heading);
}


long double dwaLocalPlanner::calc_to_obs_cost(std::vector<Node>& traj, std::vector<Node>& obs_list) {   
    long double cost;
    long double min_cost = 100;
    if(obs_list.size() == 0) {
        cost = 1;
        return cost;
    }
    else {   
        for(int j =0; j < traj.size(); j++) { 
            for(int i=0; i< obs_list.size(); i++) {
                long double dist = sqrt(pow(traj[j].x - obs_list[i].x,2) + pow(traj[j].y - obs_list[i].y,2));
                min_cost = std::min(min_cost,dist);
            }
        }
        cost = 1 - (1/min_cost);
        return roundd(cost);
    }
}


void dwaLocalPlanner::dwa_generate_trajectory(double& velocity, double& steering, std::vector<Node>& forw, std::vector<Node>& back) {
    if (track_point_recieved){
        if(current_direction) { 
            curr_x = wc_global.wheel_center.x;
            curr_y = wc_global.wheel_center.y;
        }
        else { 
            curr_x = cg_global.geometric_center.x;
            curr_y = cg_global.geometric_center.y;
        }
    }
    else{
        curr_x = current_pose.x;
        curr_y = current_pose.y;
    }
    
    for (double t = 0; t < TIME; t += DT) {
        double beta_angle = std::atan(LR*std::tan(steering)/LENGTH);
        double theta_dot = velocity * std::cos(beta_angle) * std::tan(steering)*t/LENGTH;
        double yaw = current_pose.orien + theta_dot;
        
        double x_forw = curr_x + (velocity * std::cos(yaw + beta_angle) * t);
        double y_forw = curr_y + (velocity * std::sin(yaw + beta_angle) * t);
        Node temp_pt_forw = Node(x_forw, y_forw, yaw);
        forw.push_back(temp_pt_forw);

        double x_back = curr_x - (velocity * std::cos(yaw + beta_angle) * t);
        double y_back = curr_y - (velocity * std::sin(yaw + beta_angle) * t);
        Node temp_pt_back = Node(x_back, y_back, yaw);
        back.push_back(temp_pt_back);    
    }

    for(int i = 1; i < forw.size(); i++) {
        double yaw_temp_forw = std::atan2(
            forw[i].y - forw[i-1].y, forw[i].x - forw[i-1].x);
        double yaw_temp_back = std::atan2(
            back[i].y - back[i-1].y, back[i].x - back[i-1].x);
        forw[i].orien = yaw_temp_forw;
        back[i].orien = yaw_temp_back;
    }

}

void dwaLocalPlanner::gen_localpath(int ind, int endInd, std::vector<Node> gb_path,std::vector<Node>& path, std::vector<Node>& path_vel) {
    path.clear();
    path_vel.clear();
    int nPtsLocalPath = 9;
    int nPtsLocalPathback = 10;
    int nPtsLocalPathfront = 20;
    if(corien_pathReceived) {
        nPtsLocalPath = 20;
    }
    else {
        nPtsLocalPath = 20;
    }
    for (int i = ind;
        i <= std::min(ind + nPtsLocalPath, endInd);
        i++)  {
        path.push_back(gb_path[i]); 
    }
    int lastPt = std::min(ind + nPtsLocalPathfront, endInd);
    int stPt = std::max(0,ind - nPtsLocalPathback);
    if(lastPt-stPt<30){
        nPtsLocalPathback = 30-(lastPt-stPt);
    }   
    for (int i=std::max(0,ind - nPtsLocalPathback);i<std::min(ind + nPtsLocalPathfront, endInd);i++){
        path_vel.push_back(gb_path[i]);
    }
    
    if(path.empty()) {
        ROS_DEBUG_THROTTLE(60,"local_planner/plan : path not generated");
    }
}


float dwaLocalPlanner::calc_distance(Node n1, Node n2) {
    return sqrt(pow(n2.x - n1.x, 2) + pow(n2.y - n1.y, 2));
}

std::vector<Node> dwaLocalPlanner::local_path_planner(Node wc, Node cg, Node dw, Node currLoc, int half_path_point, bool& half_path, bool recoverypathRecieved, std::vector<Node>& path_vel) {
    std::vector<Node> local_path;
    std::vector<Node> localPathrear;
    std_msgs::Float32MultiArray msg;
    std::string str1, str2;
    str1 = "set tracking point to drive wheel";
    str2 = "set tracking point to rear wheel";
    Node stop;
    std::string state;
    ind =0;
    if ((globalPath.size() != 0 && !KdTreeGenerated))  {
        goal = globalPath[globalPath.size() -1];
        if(half_path){
            for(int i = 0; i<half_path_point; i++){
                first_half_path.push_back(globalPath[i]);
            }
            for(int i = half_path_point; i<globalPath.size() - 1; i++){
                second_half_path.push_back(globalPath[i]);
            }
            gb_path = first_half_path;
            first_halfendInd = genKDTree(ind, &kdt, first_half_path);
            switch_direction = globalPath[half_path_point];
            endInd = first_halfendInd;
            path_end_size = 2;
        }
        else{
            gb_path = globalPath; 
            endInd = genKDTree(ind, &kdt, globalPath);
        }
        KdTreeGenerated = true;
    }

    if (KdTreeGenerated && ind < endInd && current_pose.x != -12342245265) {
      
        if(!track_point_recieved){
            cg = currLoc;
            dw = currLoc;
            wc = currLoc;

        }
        if(current_direction == true) {
            ind = kdt.nnSearch(KDT::MyPoint(cg.x, cg.y));
            rear_ind = kdt.nnSearch(KDT::MyPoint(wc.x, wc.y));  
            stop = dw; 
            currentIndex = ind;
        }
        else {
            ind = kdt.nnSearch(KDT::MyPoint(cg.x, cg.y)); 
            rear_ind = kdt.nnSearch(KDT::MyPoint(dw.x, dw.y)); 
            stop = wc;
            currentIndex = ind;
        }
        ros::param::get("/PP/next_goal", NEXT_GOAL);
        if(ind == 0)
            ros::param::set("/start",true);
        else
            ros::param::set("/start",false);
        gen_localpath(ind, endInd, gb_path, local_path, path_vel);
     
    }
    float distance_to_switch = calc_distance(cg, switch_direction);
    if(distance_to_switch<=2){
        ros::param::set("/PP/direction_switch", true);
    }
    else {
        ros::param::set("/PP/direction_switch", false);
    }

    Node pt_a, pt_b, pt_final, pt_path;
    pt_final = globalPath[half_path_point];
    pt_path = globalPath[half_path_point - 2];
    pt_a.x = pt_final.x + cos(pt_final.orien + PI/2);
    pt_a.y = pt_final.y + sin(pt_final.orien + PI/2);
    pt_b.x = pt_final.x - cos(pt_final.orien + PI/2);
    pt_b.y = pt_final.y - sin(pt_final.orien + PI/2);
    bool point_crossed = false;
     
    double fork_direction = (dw.x - pt_a.x)*(pt_b.y-pt_a.y) - (dw.y - pt_a.y)*(pt_b.x - pt_a.x);
    double point_direction = (pt_path.x - pt_a.x)*(pt_b.y-pt_a.y) - (pt_path.y - pt_a.y)*(pt_b.x - pt_a.x);
    if((fork_direction*point_direction)<0) {
        point_crossed = true;
    }

    if(distance_to_switch<=1 && half_path && point_crossed){
        kdt.clear();
        second_halfendInd = genKDTree(0, &kdt, second_half_path);
        endInd = second_halfendInd; 
        half_path = false;
        gb_path = second_half_path;
        ros::param::set("PP/direction", !current_direction);  
        path_end_size = 5;
        switch_direction.x = 123;           
        switch_direction.y = 123;  
        KdTreeGenerated = true;        
        
    }
    float distance_to_end = calc_distance(stop, goal);
    ros::param::get("/state", state);    
    return local_path;
}


void dwaLocalPlanner::publish_local_path(std::vector<Node>& path) {
    nav_msgs::Path path_pub;
    path_pub.header.frame_id = "map";
    path_pub.header.stamp = ros::Time::now();
    for(int i=0; i<path.size(); i++)
    {
        geometry_msgs::PoseStamped pt;
        pt.header.frame_id = "map";
        pt.header.stamp = ros::Time::now();

        pt.pose.position.x = path[i].x;
        pt.pose.position.y = path[i].y;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, path[i].orien);
        pt.pose.orientation = tf2::toMsg(quat);
        path_pub.poses.push_back(pt);
    }
    localpath_publisher.publish(path_pub);

}


int dwaLocalPlanner::argMaxVel(std::vector<double>& list) {
    std::vector<double>::iterator max = std::max_element(list.begin(), list.end());
    int argmaxVal = distance(list.begin(), max); // absolute index of max
    return argmaxVal;
}


long double dwaLocalPlanner::roundd(long double cost) {
    cost *= std::pow(10, NUM_DIGIT);
    cost  = std::round(cost);
    cost /= std::pow(10, NUM_DIGIT);
    return cost;
}


int dwaLocalPlanner::argMinVel(std::vector<double>& list) {
    std::vector<double>::iterator min = std::min_element(list.begin(), list.end());
    int argminVal = distance(list.begin(), min); // absolute index of max
    return argminVal;
}

std::vector<Node> dwaLocalPlanner::main_dwa_loop() {
    std::vector<std::vector<Node>> trajectory_forward;
    std::vector<std::vector<Node>> trajectory_backward;
    std::vector<std::vector<Node>> trajectory;

    std::vector<double> final_cost_list_forward;
    std::vector<double> final_cost_list_backward;
    std::vector<double> final_cost_list;
    ros::param::get("/PP/direction", current_direction);
    
    if((pathReceived || corien_pathReceived || recovery_pathReceived) && startposereceived) {
        
        localPath.clear();
        path_vel.clear();

        localPath = local_path_planner(wc, cg, dw, current_pose, half_path_point, half_path, recovery_pathReceived, path_vel);
        if(localPath.empty()){
            std::vector<Node> mt;
            return mt;
        }
        int track_point = std::min((int)localPath.size()-1, 9);
        localGoal = localPath[track_point];
        publish_local_path(localPath);       
        float distEndPoint;
        if(current_direction){
            distEndPoint = calc_distance(dw,globalPath.back());
        //    distEndPoint  = sqrt(pow(dw.x - globalPath[globalPath.size() - 1].x,2)+pow(dw.y - globalPath[globalPath.size() - 1].y,2));
        }
        else {
            distEndPoint = calc_distance(wc,globalPath.back());
        //    distEndPoint  = sqrt(pow(wc.x - globalPath[globalPath.size() - 1].x,2)+pow(wc.y - globalPath[globalPath.size() - 1].y,2));  
        }

        if (distEndPoint>1 && !corien_pathReceived){
            for(int i=0; i<steering_angle.size(); i++) {   
                std::vector<Node> traj_forw;
                std::vector<Node> traj_back;
                dwa_generate_trajectory(VELOCITY, steering_angle[i], traj_forw, traj_back); 
                
                if(current_direction) {
                    long double goal_cost_forw  = calc_to_goal_cost(traj_forw, localGoal);
                    long double obs_cost_forw   = calc_to_obs_cost(traj_forw, obstacles);
                    long double final_cost_forw = roundd(GOAL_COST_GAIN * goal_cost_forw + OBS_COST_GAIN * obs_cost_forw);// + SPEED_COST_GAIN * speed_cost_forw);
                    final_cost_list_forward.push_back(final_cost_forw);
                    trajectory_forward.push_back(traj_forw);
                }
                else {
                    long double goal_cost_back = calc_to_goal_cost(traj_back, localGoal);
                    long double obs_cost_back =  calc_to_obs_cost(traj_back, obstacles);
                    long double final_cost_back = roundd(GOAL_COST_GAIN * goal_cost_back + OBS_COST_GAIN * obs_cost_back);// + SPEED_COST_GAIN * speed_cost_back);
                    final_cost_list_backward.push_back(final_cost_back);
                    trajectory_backward.push_back(traj_back);
                }
            }
            
            if (current_direction) {
                stopping_node.x = dw_global.driving_wheel.x;
                stopping_node.y = dw_global.driving_wheel.y;
                final_cost_list = final_cost_list_forward;
                trajectory = trajectory_forward;
                int best_ind = argMaxVel(final_cost_list);
                return trajectory[best_ind];
            }

            else {
                stopping_node.x = wc_global.wheel_center.x;
                stopping_node.y = wc_global.wheel_center.y;
                final_cost_list = final_cost_list_backward;
                trajectory = trajectory_backward;
                int best_ind = argMaxVel(final_cost_list);
                return trajectory[best_ind];
            }  
        }
        
        else{
            return localPath;
        }

    }
    else {
        std::vector<Node> empty;
        return empty;
    }
}


void dwaLocalPlanner::driver() {
    std_msgs::Float32MultiArray msg;
    ros::param::get("/state", state);
    bool safety_off = false;
    // ros::param::get("/safety_off", safety_off);
    bool pes = false;
    bool ces = false;

    std::vector<Node> best_traj = main_dwa_loop();
    if(!best_traj.empty()) {
        // vp_dwa.velocity_points(path_vel,best_traj,globalPath,current_pose,false,false,feedbackVelocity);
        float pathPointDensityGlobal = calc_distance(globalPath[0], globalPath[1]);
        float pathLengthRemaining = pathPointDensityGlobal*(endInd - 10 -ind);
        std_msgs::Float32 pathLenMsg;
        pathLenMsg.data = pathLengthRemaining;
        pathLengthPub.publish(pathLenMsg); 
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        bool end_of_traj =false;
        ros::param::get("PP/direction", current_direction);
        for(int i=0; i<best_traj.size(); i++)
        {
            geometry_msgs::PoseStamped pt;
            pt.header.frame_id = "map";
            pt.header.stamp = ros::Time::now();

            pt.pose.position.x = best_traj[i].x;
            pt.pose.position.y = best_traj[i].y;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, best_traj[i].orien);
            pt.pose.orientation = tf2::toMsg(quat);
            path.poses.push_back(pt);
        }

        if (current_direction){
            stopping_node.x = dw_global.driving_wheel.x;
            stopping_node.y = dw_global.driving_wheel.y;
        }
        else{
            stopping_node.x = wc_global.wheel_center.x;
            stopping_node.y = wc_global.wheel_center.y;
        }
    
        Node goal_final = globalPath.back();
        
        double dist = (double)calc_distance(stopping_node, goal_final);
        // double dist = sqrt(pow(stopping_node.x - goal_final.x,2) + pow(stopping_node.y - goal_final.y,2));

        int endPt, endPtPath;
        if(corien_pathReceived){
            endPt = 1;
            endPtPath = 2;
        }
        else {
            endPt = 9;
            endPtPath = 10;
        }
        Node pt_a, pt_b, pt_final, pt_path;
        pt_final =  globalPath[globalPath.size() - endPt];
        pt_path = globalPath[globalPath.size() - endPtPath];
        pt_a.x = pt_final.x + cos(pt_final.orien + PI/2);
        pt_a.y = pt_final.y + sin(pt_final.orien + PI/2);
        pt_b.x = pt_final.x - cos(pt_final.orien + PI/2);
        pt_b.y = pt_final.y - sin(pt_final.orien + PI/2);
        bool fork_crossed = false;
        goalRegionThreshold = 1;
        if(dist<goalRegionThreshold){ 
            double fork_direction = (stopping_node.x - pt_a.x)*(pt_b.y-pt_a.y) - (stopping_node.y - pt_a.y)*(pt_b.x - pt_a.x);
            double point_direction = (pt_path.x - pt_a.x)*(pt_b.y-pt_a.y) - (pt_path.y - pt_a.y)*(pt_b.x - pt_a.x);
            if((fork_direction*point_direction)<0) {
                fork_crossed = true;
            }
        }
        if (dist<goalRegionThreshold && fork_crossed){
            ros::param::set("PP/path_done", true);
            if(recovery_pathReceived) {
                ROS_INFO("DWA_planner/plan : recovery path finished");
                ros::param::set("PP/end_of_recovery", true);
                ros::param::set("PP/end_of_trajectory", true);
                ros::param::set("PP/recovery_activated", false);
                ros::param::set("PP/path_recieved", false);
            }
            else {
                ROS_INFO("DWA_planner/plan : global path finished");
                ros::param::set("PP/end_of_trajectory", true);
            }
            ros::param::set("PP/corien", false);
            ros::param::set("PP/recovery_velocity", false);
            ros::param::set("PP/path_recieved", false);
            ROS_INFO("DWA_planner/plan : Reached goal");
            end_of_traj = true;
            kdt.clear();
            KdTreeGenerated = false;
            msg.data.clear();
            msg.data.push_back(0);
            vel_publisher.publish(msg);
            globalPath.clear();
            best_traj.clear();
            pathReceived = false;
            recovery_pathReceived = false;
            corien_pathReceived = false;
        }
        else{
            msg.data.clear();
            // ros::param::get("/emergency_stop", es);
            ros::param::get("/PP/manual", pes);
            ros::param::get("/CT/manual", ces);
            bool curve = false;
            bool curvature = false;
            velocity vel1;
            velocity vel(scanMsg, current_direction, steeringAngle, curvature);
            if(pes||ces){
                msg.data.clear();
                ROS_INFO("DWA_planner/plan : emergency stop activated");
                msg.data.push_back(0);
                vel_publisher.publish(msg);
                esPose = current_pose;
                esPose.parent = 1;
                esPose.ind = ind;
            }
            else {
                float chosen_velocity = 0;
                
                if(safety_off || scanMsg.ranges.empty()){
                    vel1.getVelocity(path_vel, curve);
                }
                else{
                    vel1.getVelocity(path_vel, curvature);
                    vel.getVelocity(path_vel, curve);
                }
                chosen_velocity = path_vel[0].velocity;
                if(chosen_velocity == 0) {
                    ROS_INFO("encountered obstacle");
                    esPose = current_pose;
                    esPose.parent = 1;
                    esPose.ind = ind;
                }
                else {
                    float pathPointDensity = calc_distance(best_traj[0], best_traj[1]);
                    float pathLengthEnd = pathPointDensity*(endInd - 10 -ind);
                    float pathLengthFromEs = pathPointDensity*(abs(ind-esPose.ind));
                    float pathLengthFromStart = pathPointDensity*(ind);
                    float vi = 1;
                    float vf = 5;
                    float distanceToMaxAcc = 4;
                    ros::param::get("/PP/maxdist", distanceToMaxAcc);
                    // float distanceToMaxAcc = 4;
                    float a = (vf*vf - vi*vi)/(2*distanceToMaxAcc);
                    if(pathLengthEnd<4){
                        // ROS_INFO("path length from end : %f", pathLengthEnd);
                        if(pathLengthEnd<0.5)
                           chosen_velocity = std::min(chosen_velocity, (float)1);
                        else{
                            if(!curve && current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)pathLengthEnd);
                            if(!curve && !current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)1);
                            if(curve) {
                                chosen_velocity = std::min(chosen_velocity, (float)1);
                            }
                        }
                    }
                    else if(pathLengthFromEs<5){
                        // ROS_INFO("path length from ES : %f", pathLengthFromEs);
                        if(pathLengthFromEs<0.5)
                            chosen_velocity = std::min(chosen_velocity, (float)1);
                        else{
                            if(!curve && current_direction){
                                // float vcurrent = sqrt(2*pathLengthFromEs*a - vi*vi);
                                // chosen_velocity = std::min(chosen_velocity, vcurrent);
                                // chosen_velocity = std::min(chosen_velocity, 5*(float)pathLengthFromEs);
                                chosen_velocity = std::min(chosen_velocity, (float)5);
                            }
                            if(!curve && !current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)2);
                            if(curve && current_direction){
                                chosen_velocity = std::min(chosen_velocity, (float)1.5);
                            }
                            if(curve && !current_direction){
                                chosen_velocity = std::min(chosen_velocity, (float)1);
                            }
                        }
                    }
                    else if(pathLengthFromStart<5){
                        // ROS_INFO("path length from start : %f", pathLengthFromStart);
                        if(pathLengthFromStart<0.5)
                            chosen_velocity = std::min(chosen_velocity, (float)1);
                        else{
                            if(!curve && current_direction){
                                // float vcurrent = sqrt(2*pathLengthFromStart*a - vi*vi);
                                // chosen_velocity = std::min(chosen_velocity, vcurrent);
                                // chosen_velocity = std::min(chosen_velocity, 5*(float)pathLengthFromStart);
                                chosen_velocity = std::min(chosen_velocity, (float)5);

                            }
                            if(!curve && !current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)2);
                            if(curve && current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)1.5);
                            if(curve && !current_direction)
                                chosen_velocity = std::min(chosen_velocity, (float)1);
                        }
                    }
                    else {
                        if(!curve && current_direction)
                            chosen_velocity = std::min(chosen_velocity, (float)5);
                        if(!curve && !current_direction)
                            chosen_velocity = std::min(chosen_velocity, (float)2);
                        if(curve){
                            chosen_velocity = std::min(chosen_velocity, (float)1.5);
                        }
                        // if(!current_direction) {
                        //     chosen_velocity = std::min((float)2,  path_vel[0].velocity);
                        // }
                        // if(curve || !current_direction)
                        //     chosen_velocity = std::min(path_vel[0].velocity, (float)1);
                        // else 
                        //     chosen_velocity = std::min(path_vel[0].velocity, pathLengthEnd);
                        // }
                        esPose.parent = -1;
                    }
                }
                ROS_INFO("velocity is %f", chosen_velocity);
                msg.data.push_back(chosen_velocity);
                vel_publisher.publish(msg);
            }
        }
        double CT_dist = sqrt(pow(cg_global.geometric_center.x - (localPath.front()).x,2) + pow(cg_global.geometric_center.y - (localPath.front()).y,2));
        if (state != "ManualControl"){
            if (CT_dist > 3){
                ros::param::set("/CT/manual",true);
                end_of_traj = true;
            }
        }
        if (!end_of_traj){
            path_publisher.publish(path);
            obstacles.clear();
        }
    }
    else {
        ROS_INFO("DWA_planner/plan : stopping the vehicle no dwa traj found");
        end_of_traj = true;
        kdt.clear();
        KdTreeGenerated = false;
        msg.data.clear();
        msg.data.push_back(0);
        vel_publisher.publish(msg);
        globalPath.clear();
        best_traj.clear();
        pathReceived = false;
        recovery_pathReceived = false;
        corien_pathReceived = false;
    } 
}