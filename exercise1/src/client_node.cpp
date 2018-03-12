#include "ros/ros.h"
#include "exercise1/Reverse.h"
#include <stdlib.h> 

//#include <cstdlib>
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "our_client_node");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<exercise1::Reverse>("our_service_node/string");

  exercise1::Reverse srv;
  std::stringstream ss;

  ss << "RInS is the best course in Mordor";

  
  for(int i = 0; i < 10; i++){
	srv.request.tabela[i] = rand() % 10 + 1;
  }

  ros::service::waitForService("our_service_node/string", 1000);

  //ROS_INFO("Sending: %s", srv.request.content.c_str());

  if (client.call(srv))
  {
   
    ROS_INFO("The service returned: %d", srv.response.result);
    for(int i = 0; i < 10; i++){
	ROS_INFO("%d ", srv.request.tabela[i]);
    }
    
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
