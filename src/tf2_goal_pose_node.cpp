#include <chrono>
#include <memory>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_ros/transform_broadcaster.h"


class TF2GoalPoseNode : public rclcpp::Node
{
public:

    TF2GoalPoseNode()
        : Node("tf2_goal_pose_node")
    {
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id = "odom";

        map_to_odom_.transform.translation.x = 0.0;
        map_to_odom_.transform.translation.y = 0.0;
        map_to_odom_.transform.translation.z = 0.0;

        map_to_odom_.transform.rotation.x = 0.0;
        map_to_odom_.transform.rotation.y = 0.0;
        map_to_odom_.transform.rotation.z = 0.0;
        map_to_odom_.transform.rotation.w = 1.0;

        odom_to_base_link_.header.frame_id = "odom";
        odom_to_base_link_.child_frame_id = "base_link";

        odom_to_base_link_.transform.translation.x = 0.0;
        odom_to_base_link_.transform.translation.y = 0.0;
        odom_to_base_link_.transform.translation.z = 0.0;

        odom_to_base_link_.transform.rotation.x = 0.0;
        odom_to_base_link_.transform.rotation.y = 0.0;
        odom_to_base_link_.transform.rotation.z = 0.0;
        odom_to_base_link_.transform.rotation.w = 1.0;


        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,std::bind(&TF2GoalPoseNode::goalCallback,this,std::placeholders::_1));

        timer_ =this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&TF2GoalPoseNode::timerCallback,this));


        RCLCPP_INFO(this->get_logger(),"TF2 Goal Pose Node Started");
    }


private:


    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Goal received");

        odom_to_base_link_.transform.translation.x = 3.0;
        odom_to_base_link_.transform.translation.y = 2.0;
        odom_to_base_link_.transform.translation.z = 0.0;


        double yaw = M_PI / 4.0;

        tf2::Quaternion fixed_quaternion;

        fixed_quaternion.setRPY(
            0.0,
            0.0,
            yaw
        );

        odom_to_base_link_.transform.rotation.x =fixed_quaternion.x();

        odom_to_base_link_.transform.rotation.y =fixed_quaternion.y();

        odom_to_base_link_.transform.rotation.z =fixed_quaternion.z();

        odom_to_base_link_.transform.rotation.w =fixed_quaternion.w();

        tf2::Transform goal;

        goal.setOrigin(
            tf2::Vector3(
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z
            )
        );


        tf2::Quaternion goal_quaternion;

        goal_quaternion.setX(msg->pose.orientation.x);

        goal_quaternion.setY(msg->pose.orientation.y);

        goal_quaternion.setZ(msg->pose.orientation.z);

        goal_quaternion.setW(msg->pose.orientation.w);

        goal.setRotation(goal_quaternion);

        tf2::Transform odom_to_base;

        odom_to_base.setOrigin(
            tf2::Vector3(
                odom_to_base_link_.transform.translation.x,
                odom_to_base_link_.transform.translation.y,
                odom_to_base_link_.transform.translation.z
            )
        );


        tf2::Quaternion odom_quaternion;

        odom_quaternion.setX(odom_to_base_link_.transform.rotation.x);

        odom_quaternion.setY(odom_to_base_link_.transform.rotation.y);

        odom_quaternion.setZ(odom_to_base_link_.transform.rotation.z);

        odom_quaternion.setW(odom_to_base_link_.transform.rotation.w);

        odom_to_base.setRotation(odom_quaternion);


        tf2::Transform map_to_odom;

        map_to_odom = goal * odom_to_base.inverse();

        map_to_odom_.transform.translation.x = map_to_odom.getOrigin().x();

        map_to_odom_.transform.translation.y = map_to_odom.getOrigin().y();

        map_to_odom_.transform.translation.z = map_to_odom.getOrigin().z();

        tf2::Quaternion q;

        q = map_to_odom.getRotation();

        map_to_odom_.transform.rotation.x = q.x();

        map_to_odom_.transform.rotation.y = q.y();

        map_to_odom_.transform.rotation.z = q.z();

        map_to_odom_.transform.rotation.w = q.w();

        goal_received_ = true;


        RCLCPP_INFO(
            this->get_logger(),
            "Goal: x=%.2f y=%.2f",
            msg->pose.position.x,
            msg->pose.position.y
        );
    }


    void timerCallback()
    {
        auto now =this->get_clock()->now();

        map_to_odom_.header.stamp = now;

        odom_to_base_link_.header.stamp = now;


        if (!goal_received_)
        {
            tf_broadcaster_->sendTransform(map_to_odom_);

            tf_broadcaster_->sendTransform(odom_to_base_link_);

            return;
        }

        tf_broadcaster_->sendTransform(map_to_odom_);

        tf_broadcaster_->sendTransform(odom_to_base_link_);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped map_to_odom_;

    geometry_msgs::msg::TransformStamped odom_to_base_link_;

    bool goal_received_ = false;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TF2GoalPoseNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}