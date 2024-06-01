// Include necessary headers for ROS and message types
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Define the class that will handle pose scaling
class ScaleTransformNode {
public:
    ScaleTransformNode() {
        ros::NodeHandle private_nh("~");  // Create a private NodeHandle to manage node-specific parameters

        double scale;
        // Retrieve the scaling factor from the parameter server with a default value
        private_nh.param("scale", scale, 3.0);

        std::string input_topic, output_topic;
        // Retrieve the input topic name from the parameter server or default to "/phantom/pose"
        private_nh.param<std::string>("input_topic", input_topic, "/phantom/pose");
        // Retrieve the output topic name from the parameter server or default to "/scaled_pose"
        private_nh.param<std::string>("output_topic", output_topic, "/scaled_pose");

        // Subscribe to the input topic with a queue size of 10 and bind the callback method
        sub = nh.subscribe(input_topic, 10, &ScaleTransformNode::poseCallback, this);
        // Advertise the output topic with a queue size of 10
        pub = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 10);

        // Store the scale factor in a member variable
        this->scale_factor = scale;
    }

    // Callback method to process incoming pose messages
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped scaled_pose = *msg;  // Make a copy of the incoming pose message
        // Scale the position components of the pose
        scaled_pose.pose.position.x *= scale_factor;
        scaled_pose.pose.position.y *= scale_factor;
        scaled_pose.pose.position.z *= scale_factor;

        // Publish the scaled pose to the output topic
        pub.publish(scaled_pose);

        // Log the scaled pose values for verification
        ROS_INFO("Published scaled pose: x=%f, y=%f, z=%f",
                 scaled_pose.pose.position.x, scaled_pose.pose.position.y, scaled_pose.pose.position.z);
    }

private:
    ros::NodeHandle nh;  // ROS NodeHandle to manage communication with ROS system
    ros::Subscriber sub; // Subscriber object to receive pose messages
    ros::Publisher pub;  // Publisher object to send scaled pose messages
    double scale_factor; // Factor by which to scale the positions
};

// Main function to set up the ROS node
int main(int argc, char **argv) {
    ros::init(argc, argv, "scale_transform_node");  // Initialize the ROS node
    ScaleTransformNode node;  // Create an instance of the ScaleTransformNode
    ros::spin();  // Enter a loop, pumping callbacks
    return 0;  // Exit the program
}
