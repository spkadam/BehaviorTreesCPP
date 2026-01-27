#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace std::chrono_literals;

// 1st way to initialize a Leaf Node: Node Class (recommended)
class ApproachObject : public BT::SyncActionNode 
{
public:
    explicit ApproachObject(const std::string &name) : BT::SyncActionNode(name, {}) 
    {

    }

    BT::NodeStatus tick() override
    {
        std::cout << "Approach Object: " << this->name() << std::endl;

        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::SUCCESS;
    }
};
// 2nd way to initialize a Leaf Node: Function
BT::NodeStatus CheckBattery()
{
    std::cout << "Battery OK" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// 3rd way to initialize a Leaf Node: Custom Class method
class GripperInterface
{
public:
    GripperInterface()
    {

    }
    BT::NodeStatus open()
    {   
        _open = true;
        std::cout << "Gripper Open" <<  std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus close()
    {
        _open = false;
        std::cout << "Gripper Close" <<  std::endl;
        return BT::NodeStatus::SUCCESS;
    }
private:
    bool _open;
};

int main() 
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));
    // Create Tree
    auto tree = factory.createTreeFromFile("./../simple_bt.xml");
    
    // Enable Groot2 monitoring
    BT::PublisherZMQ publisher_zmq(tree);
    
    // Execute the Tree
    tree.tickRoot();    

    return 0;
}