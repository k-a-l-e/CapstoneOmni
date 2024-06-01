#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"

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
    struct OmniState
    {
        hduVector3Dd position; // 3x1 vector of position
        hduVector3Dd velocity; // 3x1 vector of velocity
        hduVector3Dd inp_vel1; // 3x1 history of velocity used for filtering velocity estimate
        hduVector3Dd inp_vel2;
        hduVector3Dd inp_vel3;
        hduVector3Dd out_vel1;
        hduVector3Dd out_vel2;
        hduVector3Dd out_vel3;
        hduVector3Dd pos_hist1; // 3x1 history of position used for 2nd order backward difference estimate of velocity
        hduVector3Dd pos_hist2;
        hduVector3Dd rot;
        hduVector3Dd joints;
        hduVector3Dd force; // 3 element double vector force[0], force[1], force[2]
        double thetas[7];
        int buttons[2];
        int buttons_prev[2];
        bool lock;
        hduVector3Dd lock_pos;
    };

    class PhantomROS
    {

    public:
        ros::NodeHandle n;
        ros::Publisher pose_publisher;
        ros::Publisher joint_pub;

        ros::Publisher button_publisher;
        ros::Subscriber haptic_sub;
        std::string omni_name;
        std::string sensable_frame_name;
        std::string link_names[7];

        OmniState *state;
        tf::TransformBroadcaster br;

        void init(OmniState *s)
        {
            ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));

            // Publish on NAME/pose
            std::ostringstream stream00;
            stream00 << omni_name << "/pose";
            std::string pose_topic_name = std::string(stream00.str());
            pose_publisher = n.advertise<geometry_msgs::PoseStamped>(
                pose_topic_name.c_str(), 100);

            joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

            // Publish button state on NAME/button
            std::ostringstream stream0;
            stream0 << omni_name << "/button";
            std::string button_topic = std::string(stream0.str());
            button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(
                button_topic.c_str(), 100);

            // Subscribe to NAME/force_feedback
            std::ostringstream stream01;
            stream01 << omni_name << "/force_feedback";
            std::string force_feedback_topic = std::string(stream01.str());
            haptic_sub = n.subscribe(force_feedback_topic.c_str(), 100, &PhantomROS::force_callback, this);

            // Frame of force feedback (NAME_sensable)
            std::ostringstream stream2;
            stream2 << omni_name << "_sensable";
            sensable_frame_name = std::string(stream2.str());

            for (int i = 0; i < 7; i++)
            {
                std::ostringstream stream1;
                stream1 << omni_name << "_link" << i;
                link_names[i] = std::string(stream1.str());
            }

            state = s;
            state->buttons[0] = 0;
            state->buttons[1] = 0;
            state->buttons_prev[0] = 0;
            state->buttons_prev[1] = 0;
            hduVector3Dd zeros(0, 0, 0);
            state->velocity = zeros;
            state->inp_vel1 = zeros;  // 3x1 history of velocity
            state->inp_vel2 = zeros;  // 3x1 history of velocity
            state->inp_vel3 = zeros;  // 3x1 history of velocity
            state->out_vel1 = zeros;  // 3x1 history of velocity
            state->out_vel2 = zeros;  // 3x1 history of velocity
            state->out_vel3 = zeros;  // 3x1 history of velocity
            state->pos_hist1 = zeros; // 3x1 history of position
            state->pos_hist2 = zeros; // 3x1 history of position
            state->lock = true;
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
            state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
            state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
            state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

            state->lock_pos[0] = omnifeed->position.x;
            state->lock_pos[1] = omnifeed->position.y;
            state->lock_pos[2] = omnifeed->position.z;
        }

        void publish_omni_state()
        {
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
            joint_state.position[5] = -state->thetas[6] - M_PI;
            joint_pub.publish(joint_state);

            // Sample 'end effector' pose
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = link_names[6].c_str();
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = 0.0; // was 0.03 to end of phantom
            pose_stamped.pose.orientation.w = 1.;
            pose_publisher.publish(pose_stamped);

            if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
            {

                if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1))
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