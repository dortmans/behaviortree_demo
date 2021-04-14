#include <behaviortree_demo/pickplace_bt_nodes.h>

namespace PickPlaceNodes
{


bool WorldModel::SeeItem()
{
    return item_seen_;
}

bool WorldModel::HoldingItem()
{
    return item_holding_;
}

void WorldModel::Pick()
{
    //item_seen_ = false;
    item_holding_ = true;
}

void WorldModel::Place()
{
		//item_seen_ = true;
    item_holding_ = false;
}


BT::NodeStatus SeeItem::tick()
{    
    return worldmodel_.SeeItem() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus HoldingItem::tick()
{    
    return worldmodel_.HoldingItem() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus Pick::tick()
{ 
		
    std::cout << std::endl;
    std::cout << "Pick start" << std::endl;
    std::cout << std::endl;	
		
		waste_time_(2000);  
    worldmodel_.Pick();
    
    std::cout << std::endl;
    std::cout << "Pick done" << std::endl;
    std::cout << std::endl;
    
    //return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus Place::tick()
{
 
    std::cout << std::endl;
    std::cout << "Place start" << std::endl;
    std::cout << std::endl;
    
		waste_time_(2000);  
    worldmodel_.Place();
    
    std::cout << std::endl;
    std::cout << "Place done" << std::endl;
    std::cout << std::endl;	
      
    //return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::SUCCESS;
}


} // end PickPlaceNodes
