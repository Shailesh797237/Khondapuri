#include "plc_communication/plc_bridge.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<plc_communication::PLCBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
