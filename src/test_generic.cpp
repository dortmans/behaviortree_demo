#include <ros/ros.h>

#include <behaviortree_demo/generic_bt_nodes.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace BT;

//-----------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_generic");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	// Register palette of BT conditions and actions
  BehaviorTreeFactory factory;
  GenericNodes::RegisterNodes(factory, nh);

  // Instantiate BehaviorTree
  ROS_INFO("Creating BehaviorTree from XML text");
  
/* 
  std::string xml_text;
  if (! nh_private.getParam("xml_text", xml_text) )
  {
     ROS_ERROR("Failed to get param 'xml_text'");
  } 
  auto tree = factory.createTreeFromText(xml_text);
*/
  
  std::string bt_xml_filename; 
  if (! nh_private.getParam("bt_xml_filename", bt_xml_filename) )
  {
     ROS_ERROR("Failed to get param 'bt_xml_filename'");
  }   
  auto tree = factory.createTreeFromFile(bt_xml_filename);

	// Optionally create logfiles
	
	bool log_enabled; 
  if (! nh_private.getParam("log", log_enabled) )
  {
     ROS_ERROR("Failed to get param 'log'");
  } 	
	
  // Log state changes (~/.ros/bt_trace.fbl)
  FileLogger file_logger(tree, "bt_trace.fbl"); 	
	file_logger.setEnabled(log_enabled);
  

	// Log the execution time of each node (~/.ros/bt_trace.json)
	//MinitraceLogger minitrace_logger(tree, "bt_trace.json");
	//minitrace_logger.setEnabled(log_enabled); 
  
  // Print the tree on console
	printTreeRecursively(tree.rootNode());
		
	// Prints state changes on console
	StdCoutLogger logger_cout(tree);
     
  // Publish status changes using ZeroMQ for viewing in Groot editor
  PublisherZMQ publisher_zmq(tree);
 
	// Execute the BT
  ROS_INFO("Execute the BehaviorTree");
  
	ros::Rate rate(10); // Hz
	NodeStatus status = NodeStatus::RUNNING;
	
  while( ros::ok() && (status == NodeStatus::RUNNING) )
  {
    ros::spinOnce();
    status = tree.tickRoot();
    rate.sleep();
  }
  
  ROS_ERROR_STREAM(status);

  return 0;
}
