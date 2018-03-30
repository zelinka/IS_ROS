#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
int size_y;
int size_x;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    size_x = msg_map->info.width;
    size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
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

//        int idx_map_y = size_x * (size_y -y);
  // for (int y = 0; y < size_y; ++y) {

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

void premikanje() {
    //ros::init(argc, argv, "one_meter");
	MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int koordinate [15][2] = {
        {21,215},
        {32,234},
        {27,238},
        {24,223},
        {53,214},
        {50,220},
        {40,198},
        {33,191},
        {29,196},
        {29,169},
        {51,169},
        {61,165},
        {57,160},
        {46,190},
        {56,186}
    };

    // i < dolzina int koordinate[]
    for(int i = 0; i< 15; i++) {

		int x = koordinate[i][0];
		int y = koordinate[i][1];
		

        ROS_INFO("neki4");
    
        tf::Point pt((float)x * map_resolution, (float)(size_y-y) * map_resolution, 0.0);
        tf::Point transformed = map_transform * pt;

        ROS_INFO("Moving to (x: %f, y: %f)", transformed.x(), transformed.y());

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.w = 1;
        goal.target_pose.pose.position.x = transformed.x();
        goal.target_pose.pose.position.y = transformed.y();
        
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Hooray, the base moved");
                i++;
        }
        else{
            ROS_INFO("The base failed to move");
        }
	}
	
	return;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;
    // nav_msgs::OccupancyGrid map;

    map_sub = n.subscribe("map", 10, &mapCallback);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);

    namedWindow("Map");

    setMouseCallback("Map", mouseCallback, NULL);


    while(ros::ok()) {

        //if (!cv_map.empty()) imshow("Map", cv_map);
		
		premikanje();
        
		/*
		TODO: v premikanju dolocit kje so krogi in shranit nove cilje v new_goals tabelo
		for(int i=0; i < 3; i++)
		{
			TODO:premakni se na lokaicijo new_goals[i];
				reci nekaj;
		}
		*/
		
		//to se pol zakomentira.
		waitKey(30);

        ros::spinOnce();
    }
    return 0;

}