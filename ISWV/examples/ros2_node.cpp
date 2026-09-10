#include "iswv/iswv.hpp"
#include "iswv/transports/ros2_transport.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

class IswvExampleNode final : public rclcpp::Node {
public:
    IswvExampleNode() : rclcpp::Node("iswv_example")
    {
        hub_ = iswv::transports::RosTopicHub::create(*this);
        can0_ = std::make_unique<iswv::canopen::CanopenMaster>(hub_->bus(0));
        steering_ = std::make_unique<iswv::SteeringLayout>(*can0_);
        RCLCPP_INFO(get_logger(),
                    "CAN0 steering ready: front-left=1, rear-left=2, "
                    "rear-right=3, front-right=4; no motor command was sent");
    }

private:
    std::shared_ptr<iswv::transports::RosTopicHub> hub_;
    std::unique_ptr<iswv::canopen::CanopenMaster> can0_;
    std::unique_ptr<iswv::SteeringLayout> steering_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IswvExampleNode>());
    rclcpp::shutdown();
    return 0;
}
