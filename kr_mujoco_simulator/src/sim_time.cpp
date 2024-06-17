#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
// #include "builtin_interfaces/msg/time.hpp"
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/qos.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;

class SimTime : public rclcpp::Node
{
public:
    SimTime() : Node("sim_time")
    {
        this->declare_parameter("sim_time_factor", 0.5);
        this->k_ = this->get_parameter("sim_time_factor").as_double();

        publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
        clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
        timer_ = this->create_wall_timer(500us, std::bind(&SimTime::publish_time, this));
        init_nanoseconds_ = now().nanoseconds();

        subscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10, std::bind(&SimTime::param_event_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Clock publisher node has started");
    }

    ~SimTime()
    {
        RCLCPP_INFO(this->get_logger(), "Clock publisher node has terminated");
    }

private:
    void publish_time()
    {
        current_nanoseconds_ = k_ * (now().nanoseconds() - init_nanoseconds_);
        clock_msg->clock.sec = current_nanoseconds_ / static_cast<uint64_t>(1e9);
        clock_msg->clock.nanosec = current_nanoseconds_ % static_cast<uint64_t>(1e9);
        publisher_->publish(*clock_msg);
    }

    void param_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
    {
        if (!msg->changed_parameters.empty())
        {
            const auto &param_name = msg->changed_parameters[0].name;
            if (param_name == "sim_time_factor")
            {
                k_ = this->get_parameter("sim_time_factor").as_double();
                RCLCPP_INFO(this->get_logger(), "Simulated time constant has been set to %f ", k_);
                RCLCPP_INFO(this->get_logger(), "Simulated time will be restarted");
                init_nanoseconds_ = this->now().nanoseconds();
            }
        }
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    std::shared_ptr<rosgraph_msgs::msg::Clock> clock_msg; 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr subscription_;
    double k_;
    uint64_t init_nanoseconds_;
    uint64_t current_nanoseconds_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SimTime> node = std::make_shared<SimTime>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}