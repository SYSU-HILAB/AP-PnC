#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

using rclcpp_lifecycle::LifecycleNode;

class MyLifecycleNode : public LifecycleNode
{
   public:
    explicit MyLifecycleNode(const std::string &node_name)
        : LifecycleNode(node_name)
    {
    }

    LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");
        // Perform configuration tasks here.
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_activate() is called.");
        // Activate the node here.
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
        // Deactivate the node here.
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
        // Cleanup the node here.
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
        // Shutdown the node here.
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyLifecycleNode>("my_lifecycle_node");

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
