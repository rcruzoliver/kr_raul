// Author: Raul Cruz-Oliver
// Email: raul.cruz.oliver@gmail.com
// Date, place: January 2023, Butikkon, CH

#include "nodeCollection.hpp"

// KRStatePublisher

KRStatePublisher::KRStatePublisher(): Node("kr_state_publisher") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("kr_joints", 10, 
    std::bind(&KRStatePublisher::KRJointsCallback, this, std::placeholders::_1));
}

void KRStatePublisher::KRJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    std::vector<double> jointPositions = msg->position;
	tf2_ros::TransformBroadcaster tf_broadcaster(this);
	publishTransform("base", "link1", 0, 0, 0.035, 0, 0, jointPositions[0], tf_broadcaster, *this); 
	publishTransform("link1", "linkJ2", 0.05, 0, 0.27369999999999994, 0, jointPositions[1], 0, tf_broadcaster, *this); 
	publishTransform("link2", "link3", 0, 0, 0.274, 0, 0, jointPositions[2], tf_broadcaster, *this); 
	publishTransform("link3", "linkJ4", 0.04, 0, 0.1429, 0, jointPositions[3], 0, tf_broadcaster, *this); 
	publishTransform("link4", "link5", 0, 0, 0.271, 0, 0, jointPositions[4], tf_broadcaster, *this); 
	publishTransform("link5", "linkJ6", 0, 0, 0.10350000000000001, 0, jointPositions[5], 0, tf_broadcaster, *this); 
	publishTransform("link6", "end_effector", 0, 0, 0.108, 0, 0, jointPositions[6], tf_broadcaster, *this); 
}
  

// KRStateManager

KRStateManager::KRStateManager(): Node("kr_state_manager"), count1_(0), count2_(0),increasing1_(true), increasing2_(true){
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("kr_joints", 10);
    timer_ = this->create_wall_timer(
      	std::chrono::milliseconds(10), 
		std::bind(&KRStateManager::timer_callback, this));
	joint_name_ = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

void KRStateManager::timer_callback(){
        if (increasing1_) {count1_++; if (count1_ == 380) {increasing1_ = false;}
        } else {count1_--; if (count1_ == -659) {increasing1_ = true;}}

		if (increasing2_) {count2_++; if (count2_ == 735) {increasing2_ = false;}
        } else {count2_--; if (count2_ == -353) {increasing2_ = true;}}

		auto msg = std::make_unique<sensor_msgs::msg::JointState>();
		msg->name = joint_name_;
		msg->position = {count1_*0.001, count2_*0.0001, count1_*0.001, count2_*0.001, count1_*0.001, count2_*0.001, count1_*0.001};

      	publisher_->publish(std::move(msg));
}

