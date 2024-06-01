#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include "phantom.hpp"

class SpringEffectNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher force_pub_;

    geometry_msgs::Vector3 spring_origin_;
    double spring_constant_;
    double damping_factor_;

public:
    SpringEffectNode() {
        double x, y, z;

        // Load parameters with debug output
        nh_.param("spring_origin_x", x, 0.0);
        nh_.param("spring_origin_y", y, 0.0);
        nh_.param("spring_origin_z", z, 0.0);
        nh_.param("spring_constant", spring_constant_, 3.0);
        nh_.param("damping_factor", damping_factor_, 0.1);

        ROS_INFO("Spring Origin: (%f, %f, %f)", x, y, z);
        ROS_INFO("Spring Constant: %f, Damping Factor: %f", spring_constant_, damping_factor_);

        // Advertise and subscribe to topics
        force_pub_ = nh_.advertise<geometry_msgs::Wrench>("/omni_force_feedback", 10);
        pose_sub_ = nh_.subscribe("/phantom/pose", 10, &SpringEffectNode::poseCallback, this);
        velocity_sub_ = nh_.subscribe("/phantom/velocity", 10, &SpringEffectNode::velocityCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
        ROS_INFO("Received Pose Callback");
        geometry_msgs::Vector3 displacement;
        displacement.x = spring_origin_.x - pose->pose.position.x;
        displacement.y = spring_origin_.y - pose->pose.position.y;
        displacement.z = spring_origin_.z - pose->pose.position.z;

        geometry_msgs::Wrench spring_force;
        spring_force.force.x = spring_constant_ * displacement.x;
        spring_force.force.y = spring_constant_ * displacement.y;
        spring_force.force.z = spring_constant_ * displacement.z;

        force_pub_.publish(spring_force);
        ROS_INFO("Published Spring Force: (%f, %f, %f)", spring_force.force.x, spring_force.force.y, spring_force.force.z);
    }

    void velocityCallback(const geometry_msgs::Vector3::ConstPtr& velocity) {
        ROS_INFO("Received Velocity Callback");
        geometry_msgs::Wrench damping_force;
        damping_force.force.x = -damping_factor_ * velocity->x;
        damping_force.force.y = -damping_factor_ * velocity->y;
        damping_force.force.z = -damping_factor_ * velocity->z;

        force_pub_.publish(damping_force);
        ROS_INFO("Published Damping Force: (%f, %f, %f)", damping_force.force.x, damping_force.force.y, damping_force.force.z);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "spring_effect_node");
    ROS_INFO("Spring Effect Node Started");
    SpringEffectNode node;
    ros::spin();
    return 0;
}