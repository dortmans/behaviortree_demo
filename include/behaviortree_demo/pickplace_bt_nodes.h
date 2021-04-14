#ifndef PICKPLACE_BT_NODES_H
#define PICKPLACE_BT_NODES_H

#include <behaviortree_demo/generic_bt_nodes.h>

using namespace GenericNodes;

namespace PickPlaceNodes
{

//-------------------------------------------------------------
// WorldModel
//-------------------------------------------------------------

class WorldModel
{
  public:
    WorldModel(ros::NodeHandle& nh) : nh_(nh), item_seen_(true), item_holding_(false)
    { }
		
    bool SeeItem();
    bool HoldingItem();
    void Pick(); 
		void Place();

  private:
  	ros::NodeHandle& nh_;
    bool item_seen_;
    bool item_holding_;
};

//-------------------------------------------------------------
// Conditions
//-------------------------------------------------------------

class SeeItem : public BT::ConditionNode
{
  public:    
    SeeItem(const std::string& name, const BT::NodeConfiguration& config, WorldModel& worldmodel)
      : BT::ConditionNode(name, config), worldmodel_(worldmodel) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

		BT::NodeStatus tick() override;

  private:
    WorldModel& worldmodel_;  
};

class HoldingItem : public BT::ConditionNode
{
  public:    
    HoldingItem(const std::string& name, const BT::NodeConfiguration& config, WorldModel& worldmodel)
      : BT::ConditionNode(name, config), worldmodel_(worldmodel) {}

		static BT::PortsList providedPorts()
		{
		  return  { };
		}

		BT::NodeStatus tick() override;

  private:
    WorldModel& worldmodel_;  
};

//-------------------------------------------------------------
// Actions
//-------------------------------------------------------------

class Pick : public TimeWaster
{
  public:
  	Pick(const std::string& name, const BT::NodeConfiguration& config, WorldModel& worldmodel)
      : TimeWaster(name, config), worldmodel_(worldmodel) {}

		BT::NodeStatus tick() override;
		
  private:
    WorldModel& worldmodel_;  
};


class Place : public TimeWaster
{
  public:
  	Place(const std::string& name, const BT::NodeConfiguration& config, WorldModel& worldmodel)
      : TimeWaster(name, config), worldmodel_(worldmodel) {}

		BT::NodeStatus tick() override;
		
  private:
    WorldModel& worldmodel_;  
};


//-------------------------------------------------------------
// Node Registration
//-------------------------------------------------------------

template <class DerivedT> static
void RegisterWorldNode(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         WorldModel& worldmodel)
{
  BT::NodeBuilder builder = [&worldmodel](const std::string& name, const BT::NodeConfiguration& config) 
  {
    return std::make_unique<DerivedT>(name, config, worldmodel);
  };

  factory.registerBuilder<DerivedT>( registration_ID, builder );
}

inline void RegisterNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
	static WorldModel worldmodel(nh);

  RegisterWorldNode<SeeItem>(factory, "SeeItem", worldmodel);
  RegisterWorldNode<HoldingItem>(factory, "HoldingItem", worldmodel);
  RegisterWorldNode<Pick>(factory, "Pick", worldmodel);
  RegisterWorldNode<Place>(factory, "Place", worldmodel);
}


} // end namespace

#endif // PICKPLACE_BT_NODES_H
