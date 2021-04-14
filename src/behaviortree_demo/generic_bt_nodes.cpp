#include <behaviortree_demo/generic_bt_nodes.h>

namespace GenericNodes
{

//-------------------------------------------------------------
// Always running (Idle, Waiting)
//-------------------------------------------------------------

/*
BT::NodeStatus AlwaysRunning::tick()
{
    while ( !isHaltRequested() )
    {
        SleepMS(10);
    }

    return BT::NodeStatus::IDLE;
}
*/

BT::NodeStatus AlwaysRunning::tick()
{
		while ( true )
    {
      setStatusRunningAndYield();
    }
		
    return BT::NodeStatus::FAILURE;
}

//-------------------------------------------------------------
// Wait (delay)
//-------------------------------------------------------------

BT::NodeStatus Wait::tick()
{
    auto duration = getInput<int>("duration");
    if (!duration)
    {
        throw BT::RuntimeError( "Missing required input [duration (ms)]: ", duration.error() );
    }
    
    //std::cout << "Waiting" << std::endl;
    waste_time_(duration.value()); 
    
    return BT::NodeStatus::SUCCESS;
}

//-------------------------------------------------------------
// Print a message
//-------------------------------------------------------------
BT::NodeStatus Print::tick()
{
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "Missing required input [message]: ", msg.error() );
    }

    std::cout << "Message: " << msg.value() << std::endl;
    
    return BT::NodeStatus::SUCCESS;
}

//-------------------------------------------------------------
// Trigger a ROS service
//-------------------------------------------------------------
void Trigger::sendRequest(RequestType& request)
  {
  }

BT::NodeStatus Trigger::onResponse(const ResponseType& rep)
  {
    return rep.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

BT::NodeStatus Trigger::onFailedRequest(RosServiceNode::FailureCause failure)
  {
    ROS_ERROR("Service request failed %d", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

//-------------------------------------------------------------
// Check a condition published on a topic
//-------------------------------------------------------------
BT::NodeStatus ConditionCheck::tick()
{   	
	return status_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

//-------------------------------------------------------------
// Check if a message was received on a topic
//-------------------------------------------------------------
BT::NodeStatus MessageCheck::tick()
{   	
	bool received = status_;
  status_ = false;
  return received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


} // end GenericNodes
