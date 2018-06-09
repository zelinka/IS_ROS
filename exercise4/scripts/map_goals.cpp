//rosrun rqt_reconfigure rqt_reconfigure
//speed_lim_w = 5.7 -> star parameter
//speed_lim_v = 0.8 -> star parameter
//accel_lim_w = 2 -> star parameter
//accel_lim_v = 1 -> star parameter


#include "ros/ros.h"

#include "std_msgs/String.h"
//#include "exercise4/koordinate_markers.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <string>

# define PI 3.14159265358979323846 /* pi */

using namespace std;
using namespace cv;


string faza;
Mat cv_map;
float map_resolution = 0;
int size_y;
int size_x;
tf::Transform map_transform;

float array_ring[100][3];
float array_cylinder[100][4];
int array_counter_ring = 0;
int array_counter_cylinder = 0;

ros::Subscriber array_sub_c;
ros::Subscriber array_sub_r;

ros::Publisher goal_pub;
ros::Publisher homo_pub;
ros::Subscriber map_sub;
ros::Subscriber reg_sub;

ros::Publisher arm_pub;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int pozicije_roke [3][7] = {
    {0, 0, 1, 2, 9, 10, 3},
    {3, 3, 4, 5, 9, 10, 6},
    {6, 6, 7, 8, 9, 10, 0}
};

int which = 0;

void kovanec(){
    std_msgs::Int8 msg;

    for(int i = 0; i < 7; i++){
        msg.data = pozicije_roke[which][i];
        arm_pub.publish(msg);
        if (i < 3)
            ros::Duration(1).sleep();
        else
            ros::Duration(1.5).sleep();
    }

    which++;

    if(which>3)
        which = 0;
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    size_x = msg_map->info.width;
    size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        //ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);
    
    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

   for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        //int idx_map_y = size_x * y;
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;        

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }

}



void mouseCallback(int event, int x, int y, int, void* data) {

}

void cylinderRecieved(const geometry_msgs::Pose msg){
    //ROS_INFO("V MARKERS RECIEVED CALLBACKU");
    if( !isnan(msg.position.x) || !isnan(msg.position.y) || !isnan(msg.position.z) ) {
        array_cylinder[array_counter_cylinder][0] = msg.position.x;
        array_cylinder[array_counter_cylinder][1] = msg.position.y;
        array_cylinder[array_counter_cylinder][2] = msg.position.z; //kot
        array_cylinder[array_counter_cylinder][3] = msg.orientation.w; //barva 0 R, 1 G, 2 B, 3 Y 
        ROS_INFO("x=%f y=%f ------- array", array_cylinder[array_counter_cylinder][0], array_cylinder[array_counter_cylinder][1]);
        ROS_INFO("x=%f y=%f ------- message", msg.position.x, msg.position.y);
        array_counter_cylinder++;
    }
}

void regression_result_callback(const geometry_msgs::Pose msg){
    //ros::init(argc, argv, "one_meter");

    ROS_INFO("REGRESSION RESULT CALLBACK");

    int len = msg.position.x;
    int reg_goals[len];

    
    if (len >= 1)
        reg_goals[0] = static_cast<int>(msg.orientation.x);
    if (len >= 2)
        reg_goals[1] = static_cast<int>(msg.orientation.y);
    if (len >= 3)
        reg_goals[2] = static_cast<int>(msg.orientation.z);
    if (len >= 4)
        reg_goals[3] = static_cast<int>(msg.orientation.w);

    ROS_INFO("Barve po vrsti: %d %d %d", reg_goals[0], reg_goals[1], reg_goals[2]);

	MoveBaseClient ac("move_base", true);


    while(!ac.waitForServer(ros::Duration(5.0))){
        //ROS_INFO("Waiting for the move_base action server to come up");
    }

    // i < dolzina int koordinate[]
    for(int i = 0;i < len && i < 4;i++) {
        
        int barva = reg_goals[i];
        ROS_INFO("grem pozdravljat cilindre %d od %d", i, array_counter_cylinder);
		
        for(int j = 0;j < array_counter_cylinder;j++){

            if(barva == array_cylinder[j][3]){

                float x = array_cylinder[j][0];
                float y = array_cylinder[j][1];
                float kot = array_cylinder[j][2];
		
                ROS_INFO("pozdravljam iz map_goals x = %f", x);
                ROS_INFO("pozdravljam iz map_goals y = %f", y);
                ROS_INFO("pozdravljam iz map_goals barva = %f", array_cylinder[i][3]);

                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.pose.orientation.x = 0;
                goal.target_pose.pose.orientation.y = 0;
                goal.target_pose.pose.orientation.z = 1*sin(kot/2);
                goal.target_pose.pose.orientation.w = cos(kot/2);
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;

                
                ROS_INFO("grem pozdravljat");
                ac.sendGoal(goal);
                
                ac.waitForResult();


                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        //ROS_INFO("Hooray, the base moved");
                        system("rosrun sound_play say.py 'hello cyllinder'");
                        kovanec();
                        
                }
                else{
                    system("rosrun sound_play say.py 'failed to reach goal'");
                    //i++;
                    //ROS_INFO("The base failed to move");
                }
                
                if (i >= array_counter_cylinder && array_counter_cylinder != 0){
                    system("rosnode kill /map_goals");
                }
                break;
            }
        }
	}
}


void ringRecieved(const geometry_msgs::Pose msg){
    //ROS_INFO("V MARKERS RECIEVED CALLBACKU");
    array_ring[array_counter_ring][0] = msg.position.x;
    array_ring[array_counter_ring][1] = msg.position.y;
    array_ring[array_counter_ring][2] = msg.position.z;
    ROS_INFO("x=%f y=%f ------- array", array_ring[array_counter_ring][0], array_ring[array_counter_ring][1]);
    ROS_INFO("x=%f y=%f ------- message", msg.position.x, msg.position.y);
    array_counter_ring++;
}

void premikanje() {
    //ros::init(argc, argv, "one_meter");
	MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        //ROS_INFO("Waiting for the move_base action server to come up");
    }

    /* mapa2
    int koordinate [14][3] = {
        {27,219, -90},
        {25,196, -90},
        {22,181, 0},
        {40,168, -120},
        {60,165, 90},
        {60,165, 160},
        {60,165, -130},
        {52,183, -30},
        {52,183, 0},
        {52,183, 180},
        {48,220, 30},
        {25,240, 0},
        {28,246, -140},
        {27,219, 90}
    };*/

    /*
    int koordinate [14][3] = {
        {52,183, -30},
        {52,183, 0},
        {52,183, 180},
        {48,220, 30},
        {25,240, 0},
        {28,246, -140},
        {27,219, 90},
		{27,219, 180},
        {25,196, -90},
        {22,181, 0},
        {40,168, -120},rdinate[]
    for(int i = 0;
        {60,165, 90},
        {60,165, 160},
        {60,165, -130}
    };
    */
    
    /*
    int koordinate [num_of_goals][3] = {
        {55,169, 0},
        {52,183, -30},
        {52,183, 45},
        {52,183, 180},
        {41, 198, 0},
        {41, 198, 90},
        {41, 198, 180},
        {41, 198, 270},
        {41, 198, -10},
        {55,220, 0},
        {55,220, 135},
        {34, 222, -65},
        {27,243, -45},
        {27,243, -90},
        {27,243, -140},
        {27,243, -220},
        {27,219, 90},
		{27,219, 180},
        {25,196, -90},//
        {22,181, 0},
        {32, 168, 90},
        {40,168, -120},
        {60,165, 90},
        {60,165, 160},
        {60,165, -130}
    };
    */
    int num_of_goals = 23;
    int koordinate [num_of_goals][3] = {
        {45, 230, 0},
        {45, 230, 180},
        {45, 230, 315},
        {44,268, 90},
        {44,268, 0},
        {44,268,270},
        {20,293, 0},
        {20,293, 270},
        {22,272, 250},
        {22, 272, 180},
        {22, 272, 90},
        {30, 249, 0},
        {30, 249, 90},
        {30, 249, 180},
        {18, 245, 270},
        {15, 227, 0},
        {15, 227, 90},
        {15, 227, 180},
        {18, 217, 90},
        {18, 217, 270},
        {45, 208, 0},
        {45, 208, 90},
        {45, 208, 180},
    };

    // i < dolzina int koordinate[]
    int i = 3;
    int num_of_destinations = 10;
    //float pi = 3.14159265358979323846;
    ROS_INFO("premikanje");
    while(i < num_of_destinations) {

		int x = koordinate[i][0];
		int y = koordinate[i][1];
        float kot = koordinate[i][2]*PI/180;
        int kot2 = koordinate[i][2];
		ROS_INFO("x = %d", x);
        ROS_INFO("y = %d", y);
        ROS_INFO("kot = %d", kot2);

        //ROS_INFO("neki4");
    
        tf::Point pt((float)x * map_resolution, (float)(size_y-y) * map_resolution, 0.0);
        tf::Point transformed = map_transform * pt;
        ROS_INFO("Moving to (x: %f, y: %f), i = %d", transformed.x(), transformed.y(), i);
        //ROS_INFO("Moving to (x: %d, y: %d), i = %d", x, y, i); 

        if (transformed.x() == 0.0 && transformed.y() == 0.0) {
            break;
        }

        //image_converter_23809_1522578337739
        //image_converter_3935_1522578647698
        
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 1*sin(kot/2);
        goal.target_pose.pose.orientation.w = cos(kot/2);
        goal.target_pose.pose.position.x = transformed.x();
        goal.target_pose.pose.position.y = transformed.y();

        
        //ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                //ROS_INFO("Hooray, the base moveset_manipulator_positiond");
                //system("rosrun sound_play say.py 'position'");
                //i++;
                ros::Duration(0.75).sleep();
               
        }
        else{
            //ROS_INFO("The base failed to move");
        }
        i++;
        if (i >= num_of_destinations){
            
            ROS_INFO("Killing /cylinder_marker_clustering and /cylinder_segmentation and /image_converter");
            system("rosnode kill /cylinder_marker_clustering");
            system("rosnode kill /cylinder_segmentation");
            system("rosnode kill /image_converter");
            
        }
    }
    
    //ros::spinOnce();

	return;
}

void pozdravljanje_cylinder() {
    //ros::init(argc, argv, "one_meter");
	MoveBaseClient ac("move_base", true);


    while(!ac.waitForServer(ros::Duration(5.0))){
        //ROS_INFO("Waiting for the move_base action server to come up");
    }

    // i < dolzina int koordinate[]
    int i = 0;
    while(i < array_counter_cylinder) {

        ROS_INFO("grem pozdravljat cilindre %d od %d", i, array_counter_cylinder);
		float x = array_cylinder[i][0];
		float y = array_cylinder[i][1];
        float kot = array_cylinder[i][2];
        ROS_INFO("pozdravljam iz map_goals x = %f", x);
        ROS_INFO("pozdravljam iz map_goals y = %f", y);
        ROS_INFO("pozdravljam iz map_goals barva = %f", array_cylinder[i][3]);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 1*sin(kot/2);
        goal.target_pose.pose.orientation.w = cos(kot/2);
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;

        
        ROS_INFO("grem pozdravljat");
        ac.sendGoal(goal);
        
        ac.waitForResult();


        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                //ROS_INFO("Hooray, the base moved");
                system("rosrun sound_play say.py 'hello cyllinder'");
                kovanec();
                //i++;
        }
        else{
            system("rosrun sound_play say.py 'failed to reach goal'");
            //i++;
            //ROS_INFO("The base failed to move");
        }
        i++;
        if (i >= array_counter_cylinder && array_counter_cylinder != 0){
            system("rosnode kill /map_goals");
        }
	}

	return;
}


void pozdravljanje_ring() {
    //ros::init(argc, argv, "one_meter");
	MoveBaseClient ac("move_base", true);


    while(!ac.waitForServer(ros::Duration(5.0))){
        //ROS_INFO("Waiting for the move_base action server to come up");
    }

    // i < dolzina int koordinate[]
    int i = 0;
    while(i < array_counter_ring) {

        ROS_INFO("grem pozdravljat kroge %d od %d", i, array_counter_ring);
		float x = array_ring[i][0];
		float y = array_ring[i][1];
        float kot = array_ring[i][2]; 
        ROS_INFO("pozdravljam iz map_goals x = %f", x);
        ROS_INFO("pozdravljam iz map_goals y = %f", y);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 1*sin(kot/2);
        goal.target_pose.pose.orientation.w = cos(kot/2);
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;

        
        ROS_INFO("grem pozdravljat");
        ac.sendGoal(goal);
        
        ac.waitForResult();


        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                //ROS_INFO("Hooray, the base moved");
                system("rosrun sound_play say.py 'my precious'");
                std_msgs::String msg;
                msg.data = "picture";
                homo_pub.publish(msg);
                ROS_INFO("poslano homo_pub");
                //i++;
        }
        else{
            system("rosrun sound_play say.py 'failed to reach goal'");
            //i++;
            //ROS_INFO("The base failed to move");
        }
        i++;
        if (i >= array_counter_ring && array_counter_ring != 0){
            ros::Duration(1).sleep();
            std_msgs::String nextGoal;
            nextGoal.data = "next goal";
            homo_pub.publish(nextGoal);
            faza = "regresija";

        }
	}


	return;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;
    // nav_msgs::OccupancyGrid map;
    ROS_INFO("map_goals");
    map_sub = n.subscribe("map", 10, &mapCallback);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);

    ros::NodeHandle n2;
    array_sub_c = n2.subscribe("/markers_cylinder",1000,cylinderRecieved);
    ros::NodeHandle n4;
    array_sub_r = n4.subscribe("/markers_ring",1000,ringRecieved);
    //ROS_INFO("array_sub subscriban");

    ros::NodeHandle n3;
    arm_pub = n3.advertise<std_msgs::Int8>("set_manipulator_position", 10);
    ros::NodeHandle n5;
    homo_pub = n5.advertise<std_msgs::String>("/homography_image_2", 10);
    ros::NodeHandle n6;
    reg_sub = n6.subscribe<geometry_msgs::Pose>("/regression_result",10, regression_result_callback);
    
    //namedWindow("Map");

    //setMouseCallback("Map", mouseCallback, NULL);
    faza = "pregledovanje";

    while(ros::ok()) {

        //if (!cv_map.empty()) imshow("Map", cv_map);
		if(faza.compare("pregledovanje") == 0) {
            premikanje();
            ros::spinOnce();
            ROS_INFO("ring counter %d", array_counter_ring);
            ROS_INFO("cylinder counter %d", array_counter_cylinder);
            //ROS_INFO("KONCAL Z PREISKOVANJEM> POZDRAVLJANJE");
            pozdravljanje_ring();
        } else {
            
            ROS_INFO("poslano homo_pub next goal, cakam na callback");
            ros::Duration(1).sleep();
            ros::spinOnce();
        }

        //pozdravljanje_cylinder();
        
        //break;
    }

    return 0;

}

