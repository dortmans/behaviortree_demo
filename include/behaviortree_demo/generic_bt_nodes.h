#ifndef GENERIC_BT_NODES_H
#define GENERIC_BT_NODES_H

#include "behaviortree_cpp_v3/bt_factory.h"

#include <ros/ros.h>
#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>


namespace GenericNodes
{

typedef std::chrono::high_resolution_clock::time_point TimePoint;
typedef std::chrono::milliseconds Milliseconds;

inline TimePoint Now()
{ 
        return std::chrono::high_resolution_clock::now();
};

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


//-------------------------------------------------------------
// Always running (Idle)
//-------------------------------------------------------------

/*
class AlwaysRunning : public BT::AsyncActionNode
{
  public:
    AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

    BT::NodeStatus tick() override;
};
*/

class AlwaysRunning : public BT::CoroActionNode
{
  public:
    AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

    BT::NodeStatus tick() override;
};

//-------------------------------------------------------------
// TimeWaster base class for actions that waste time
//-------------------------------------------------------------

/*
class TimeWaster : public BT::AsyncActionNode
{
  public:
    TimeWaster(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

		virtual BT::NodeStatus tick()
		{
			waste_time_(2000);
 
     	return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
		}

  protected:		 
		void waste_time_(const int ms)
		{
			const int DT = 10; // ms
			int	count = ms/DT;
    	while ( !isHaltRequested() && count-- > 0 )
    	{
      	SleepMS(DT);
    	}
		}
};
*/

class TimeWaster : public BT::CoroActionNode
{
  public:
    TimeWaster(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

		virtual BT::NodeStatus tick()
		{
			waste_time_(1000);
 
     	return BT::NodeStatus::SUCCESS;
		}
		
  protected:  
		void waste_time_(const int ms)
		{
			TimePoint initial_time = Now();
      TimePoint end_time = initial_time + Milliseconds(ms);
      
    	while ( Now() < end_time )
    	{
      	setStatusRunningAndYield();
    	}
		}
};

//-------------------------------------------------------------
// Wait for some duration
//-------------------------------------------------------------

class Wait : public TimeWaster
{
  public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
      : TimeWaster(name, config) {}

		static BT::PortsList providedPorts()
		{
		  return { BT::InputPort<int>("duration") };
		}

		BT::NodeStatus tick() override;
	
};

//-------------------------------------------------------------
// Print a message
//-------------------------------------------------------------
class Print: public BT::SyncActionNode
{
  public:
    Print(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("message") };
    }

	  BT::NodeStatus tick() override;
};


//-------------------------------------------------------------
// Trigger a service
//-------------------------------------------------------------
class Trigger: public BT::RosServiceNode<std_srvs::Trigger>
{
	public:
		Trigger( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration & conf):
				BT::RosServiceNode<std_srvs::Trigger>(handle, node_name, conf) {}

		static BT::PortsList providedPorts()
		{
		  return { };
		}

		void sendRequest(RequestType& request) override;
		BT::NodeStatus onResponse(const ResponseType& rep) override;
		BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override;
};

//-------------------------------------------------------------
// Check a condition published on a topic
//-------------------------------------------------------------
class ConditionCheck: public BT::ConditionNode
{
	public:
		ConditionCheck( ros::NodeHandle& nh, const std::string& node_name, const BT::NodeConfiguration & conf):
				BT::ConditionNode(node_name, conf)
		{
			std::string topic = getInput<std::string>("topic").value();
			sub_ = nh.subscribe(topic, 1, &ConditionCheck::callback, this);
		}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("topic") };
    }

  	BT::NodeStatus tick() override;

		void callback(const std_msgs::Bool& msg)
		{
			status_ = msg.data;
		}

	private:
		ros::Subscriber sub_;
		bool status_ = false;
};


//-------------------------------------------------------------
// Check if a message was received on a topic
//-------------------------------------------------------------
class MessageCheck: public BT::ConditionNode
{
	public:
		MessageCheck( ros::NodeHandle& nh, const std::string& node_name, const BT::NodeConfiguration & conf):
				BT::ConditionNode(node_name, conf)
		{
			std::string topic = getInput<std::string>("topic").value();
			sub_ = nh.subscribe(topic, 1, &MessageCheck::callback, this);
		}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("topic") };
    }

  	BT::NodeStatus tick() override;

		void callback(const std_msgs::Empty& msg)
		{
			status_ = true;
		}

	private:		
		ros::Subscriber sub_;
		bool status_ = false;
};

//-------------------------------------------------------------
// Node Registration
//-------------------------------------------------------------

template <class DerivedT> static
void RegisterRosNode(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& nh)
{
  BT::NodeBuilder builder = [&nh](const std::string& name, const BT::NodeConfiguration& config) 
  {
    return std::make_unique<DerivedT>(nh, name, config);
  };
  factory.registerBuilder<DerivedT>( registration_ID, builder );
}

inline void RegisterNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
	factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
	factory.registerNodeType<Wait>("Wait");
	factory.registerNodeType<Print>("Print");
	RegisterRosNode<ConditionCheck>(factory, "ConditionCheck", nh);
	RegisterRosNode<MessageCheck>(factory, "MessageCheck", nh);
  BT::RegisterRosService<Trigger>(factory, "Trigger", nh);
}

} // end namespace

#endif // GENERIC_BT_NODES_H
