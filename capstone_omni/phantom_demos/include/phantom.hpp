#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <thread>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"

std::string trim(const std::string &str, const std::string &whitespace = " \t\n\r")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

#define ROS_HD_ERROR(error, message)               \
    {                                              \
        char *buf = NULL;                          \
        size_t bufSize = 0;                        \
        FILE *ss = open_memstream(&buf, &bufSize); \
        hduPrintError(ss, &error, message);        \
        fclose(ss);                                \
        ROS_ERROR("\n%s", trim(buf).c_str());      \
        free(buf);                                 \
    }

namespace omni
{
    template <class T>
    struct StampedVector3D : public hduVector3D<T>
    {
        // hduVector3Dd vector;
        ros::Time stamp;

        StampedVector3D() : hduVector3Dd(), stamp() {}
        StampedVector3D(T x, T y, T z) : hduVector3Dd(x, y, z), stamp() {}
        explicit StampedVector3D(const T val[3]) : hduVector3Dd(val), stamp() {}
        StampedVector3D(const hduVector3Dd &B) : hduVector3Dd(B), stamp() {}
    };

    typedef StampedVector3D<double> StampedVector3Dd;
    typedef StampedVector3D<float> StampedVector3Df;

    struct OmniState
    {
        StampedVector3Dd position; // 3x1 vector of position
        StampedVector3Dd velocity; // 3x1 vector of velocity
        StampedVector3Dd inp_vel1; // 3x1 history of velocity used for filtering velocity estimate
        StampedVector3Dd inp_vel2;
        StampedVector3Dd inp_vel3;
        StampedVector3Dd out_vel1;
        StampedVector3Dd out_vel2;
        StampedVector3Dd out_vel3;
        StampedVector3Dd pos_hist1; // 3x1 history of position used for 2nd order backward difference estimate of velocity
        StampedVector3Dd pos_hist2;
        hduQuaternion rot;
        StampedVector3Dd joints;
        StampedVector3Dd force; // 3 element double vector force[0], force[1], force[2]
        double thetas[7];
        int buttons[2];
        int buttons_prev[2];
        bool lock;
        bool close_gripper;
        StampedVector3Dd lock_pos;

        StampedVector3Dd test_vel; // 3x1 vector of velocity
    };

    class PhantomROS
    {

    public:
        ros::NodeHandle n;
        ros::Publisher state_publisher;
        ros::Publisher pose_publisher;
        ros::Publisher button_publisher;
        ros::Publisher joint_publisher;
        ros::Subscriber haptic_sub;
        std::string units;
        tf::TransformListener listener;

        OmniState *state;

        void init(OmniState *s)
        {
            // get namespace for this node
            std::string ns = ros::this_node::getNamespace();

            // Publish button state on NAME/button
            std::ostringstream stream1;
            stream1 << ns << "/button";
            std::string button_topic = std::string(stream1.str());
            button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);

            // Publish on NAME/state
            std::ostringstream stream2;
            stream2 << ns << "/state";
            std::string state_topic_name = std::string(stream2.str());
            state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);

            // Subscribe to NAME/force_feedback
            std::ostringstream stream3;
            stream3 << ns << "/omni_force_feedback";
            std::string force_feedback_topic = std::string(stream3.str());
            haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

            // Publish on NAME/pose
            std::ostringstream stream4;
            stream4 << ns << "/pose";
            std::string pose_topic_name = std::string(stream4.str());
            pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);

            // Publish on NAME/joint_states
            std::ostringstream stream5;
            stream5 << ns << "/joint_states";
            std::string joint_topic_name = std::string(stream5.str());
            joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

            state = s;
            state->buttons[0] = 0;
            state->buttons[1] = 0;
            state->buttons_prev[0] = 0;
            state->buttons_prev[1] = 0;
            StampedVector3Dd zeros(0, 0, 0);
            state->velocity = zeros;
            state->inp_vel1 = zeros;  // 3x1 history of velocity
            state->inp_vel2 = zeros;  // 3x1 history of velocity
            state->inp_vel3 = zeros;  // 3x1 history of velocity
            state->out_vel1 = zeros;  // 3x1 history of velocity
            state->out_vel2 = zeros;  // 3x1 history of velocity
            state->out_vel3 = zeros;  // 3x1 history of velocity
            state->pos_hist1 = zeros; // 3x1 history of position
            state->pos_hist2 = zeros; // 3x1 history of position
            state->lock = false;
            state->close_gripper = false;
            state->lock_pos = zeros;
        }

        /*******************************************************************************
         ROS node callback.
         *******************************************************************************/
        void force_callback(const omni_msgs::OmniFeedbackConstPtr &omnifeed)
        {
            ////////////////////Some people might not like this extra damping, but it
            ////////////////////helps to stabilize the overall force feedback. It isn't
            ////////////////////like we are getting direct impedance matching from the
            ////////////////////omni anyway
            ROS_INFO("Received force feedback: (%f, %f, %f)",
                omnifeed->force.x, omnifeed->force.y, omnifeed->force.z);

            state->force[0] = omnifeed->force.x; - 0.0 * state->velocity[0];
            state->force[1] = omnifeed->force.y; - 0.0 * state->velocity[1];
            state->force[2] = omnifeed->force.z; - 0.0 * state->velocity[2];

            state->lock_pos[0] = omnifeed->position.x;
            state->lock_pos[1] = omnifeed->position.y;
            state->lock_pos[2] = omnifeed->position.z;
        }

        void publish_omni_state()
        {
            try
            {
                tf::StampedTransform transform;
                listener.lookupTransform("/base_link", "/stylus_link", ros::Time(0), transform);
                state->position.set(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                state->position.stamp = ros::Time::now();

                state->rot.v()[0] = transform.getRotation().x();
                state->rot.v()[1] = transform.getRotation().y();
                state->rot.v()[2] = transform.getRotation().z();
                state->rot.s() = transform.getRotation().w();
            }
            catch (tf::TransformException ex)
            {
                ROS_WARN_STREAM_ONCE("Phantom Driver was unable to find robot transforms. Please make sure robot_state_publisher is running.");
                ROS_ERROR_STREAM("" << ex.what());
                ros::Duration(1.0).sleep();
            }

            // // Velocity estimation
            // StampedVector3Dd vel_buff(0, 0, 0);
            // vel_buff = (state->position * 3 - 4 * state->pos_hist1 + state->pos_hist2) / 0.002;                                                                                                                  //(units)/s, 2nd order backward dif
            // vel_buff.stamp = state->position.stamp;
            // state->velocity = (.2196 * (vel_buff + state->inp_vel3) + .6588 * (state->inp_vel1 + state->inp_vel2)) / 1000.0 - (-2.7488 * state->out_vel1 + 2.5282 * state->out_vel2 - 0.7776 * state->out_vel3); // cutoff freq of 20 Hz
            // state->velocity.stamp = vel_buff.stamp;

            // // ROS_INFO_STREAM("Velocity: " << state->velocity[0] << ", " << state->velocity[1] << ", " << state->velocity[2]);
            // // ROS_INFO_STREAM("Test Vel: " << state->test_vel[0] << ", " << state->test_vel[1] << ", " << state->test_vel[2]);
            // // ROS_INFO("\n");

            // state->pos_hist2 = state->pos_hist1;
            // state->pos_hist1 = state->position;
            // state->inp_vel3 = state->inp_vel2;
            // state->inp_vel2 = state->inp_vel1;
            // state->inp_vel1 = vel_buff;
            // state->out_vel3 = state->out_vel2;
            // state->out_vel2 = state->out_vel1;
            // state->out_vel1 = state->velocity;

            hduVector3Dd vel_buff(0, 0, 0);
            vel_buff = (state->position * 3 - 4 * state->pos_hist1 + state->pos_hist2) / 0.002;                                                                                                                                      // mm/s, 2nd order backward dif
            state->velocity = (.2196 * (vel_buff + state->inp_vel3) + .6588 * (state->inp_vel1 + state->inp_vel2)) / 1000.0 - (-2.7488 * state->out_vel1 + 2.5282 * state->out_vel2 - 0.7776 * state->out_vel3); // cutoff freq of 20 Hz
            state->pos_hist2 = state->pos_hist1;
            state->pos_hist1 = state->position;
            state->inp_vel3 = state->inp_vel2;
            state->inp_vel2 = state->inp_vel1;
            state->inp_vel1 = vel_buff;
            state->out_vel3 = state->out_vel2;
            state->out_vel2 = state->out_vel1;
            state->out_vel1 = state->velocity;
            // if (state->lock == true)
            // {
            //     state->force = 0.04 * (state->lock_pos - state->position) - 0.001 * state->velocity;
            // }

            // Build the state msg
            omni_msgs::OmniState state_msg;
            // Locked
            state_msg.locked = state->lock;
            state_msg.close_gripper = state->close_gripper;
            // Position
            state_msg.pose.position.x = state->position[0];
            state_msg.pose.position.y = state->position[1];
            state_msg.pose.position.z = state->position[2];
            // Orientation
            state_msg.pose.orientation.x = state->rot.v()[0];
            state_msg.pose.orientation.y = state->rot.v()[1];
            state_msg.pose.orientation.z = state->rot.v()[2];
            state_msg.pose.orientation.w = state->rot.s();
            // Velocity
            state_msg.velocity.x = state->velocity[0];
            state_msg.velocity.y = state->velocity[1];
            state_msg.velocity.z = state->velocity[2];
            // TODO: Append Current to the state msg
            state_msg.header.stamp = ros::Time::now();
            state_publisher.publish(state_msg);

            // Publish the JointState msg
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);

            joint_state.name[0] = "base_torso_joint";
            joint_state.position[0] = -state->thetas[1];
            joint_state.name[1] = "torso_upper_arm_joint";
            joint_state.position[1] = state->thetas[2];
            joint_state.name[2] = "upper_arm_lower_arm_joint";
            joint_state.position[2] = state->thetas[3];
            joint_state.name[3] = "lower_arm_wrist_joint";
            joint_state.position[3] = -state->thetas[4] + M_PI;
            joint_state.name[4] = "wrist_tip_joint";
            joint_state.position[4] = -state->thetas[5] - 3 * M_PI / 4;
            joint_state.name[5] = "tip_stylus_joint";
            joint_state.position[5] = state->thetas[6] + M_PI;
            joint_publisher.publish(joint_state);

            // Build the pose msg
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = state_msg.header;
            pose_msg.header.frame_id = "base_link";
            pose_msg.pose = state_msg.pose;
            // pose_msg.pose.position.x /= 1000.0;
            // pose_msg.pose.position.y /= 1000.0;
            // pose_msg.pose.position.z /= 1000.0;
            pose_publisher.publish(pose_msg);

            if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
            {
                if (state->buttons[0] == 1)
                {
                    state->close_gripper = !(state->close_gripper);
                }
                if (state->buttons[1] == 1)
                {
                    state->lock = !(state->lock);
                }
                omni_msgs::OmniButtonEvent button_event;
                button_event.grey_button = state->buttons[0];
                button_event.white_button = state->buttons[1];
                state->buttons_prev[0] = state->buttons[0];
                state->buttons_prev[1] = state->buttons[1];
                button_publisher.publish(button_event);
            }
        }
    };
}