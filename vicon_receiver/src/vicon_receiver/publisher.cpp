#include "vicon_receiver/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node::SharedPtr node)
{
    position_publisher_ = node->create_publisher<vicon_receiver::msg::Position>(topic_name, 10);
    pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name + "/pose", 10);
    node_ = node;
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto node = node_.lock();
    if (node) {
        auto pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose->header.frame_id = p.segment_name;
        pose->header.stamp = node->get_clock()->now();
        pose->pose.position.x = p.translation[0];
        pose->pose.position.y = p.translation[1];
        pose->pose.position.z = p.translation[2];
        pose->pose.orientation.x = p.rotation[0];
        pose->pose.orientation.y = p.rotation[1];
        pose->pose.orientation.z = p.rotation[2];
        pose->pose.orientation.w = p.rotation[3];
        
        auto msg = std::make_unique<vicon_receiver::msg::Position>();
        msg->header = pose->header;
        msg->pose = pose->pose;
        msg->subject_name = p.subject_name;
        msg->segment_name = p.segment_name;
        msg->frame_number = p.frame_number;
        msg->translation_type = p.translation_type;
        pose_publisher_->publish(std::move(pose));
        position_publisher_->publish(std::move(msg));
    }
}
