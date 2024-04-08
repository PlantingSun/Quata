#ifndef JUMPCONTROLLER_H
#define JUMPCONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "Data.h"
#include "DeltaMechanism.h"
#include "NNInterface.h"

namespace controller{
    /* controller */
    class JumpController{
        public:
            JumpController(double l_0_,double k_spring_,double margin_,
                           double joint_kp_,double joint_kd_,double ctrl_rate_,
                           controller::NNInterface* nninterface_)
            {
                nninterface = nninterface_;
                l_0 = l_0_;
                k_spring = k_spring_;
                margin = margin_;
                joint_kp = joint_kp_;
                joint_kd = joint_kd_;
                ctrl_rate = ctrl_rate_;
                first_minimum = 0;
            }
            void Controller(controller::body_data& body);
        private:
            void QuattoRotMat(double* quat,double (*rot_mat)[3]);
            void QuattoEuler(double* quat,double& roll,double& pitch,double& yaw);
            void UpdateParam(controller::body_data& body);
            void JudgeState(controller::body_data& body);
            void SetFlyingAngle(controller::body_data& body);
            void SetLandingForce(controller::body_data& body);
            void ApplyRLNetwork(controller::body_data& body);
            double l_0,k_spring,margin;
            double joint_kp,joint_kd,ctrl_rate;
            int first_minimum;
            double input[21],output[3];

            controller::Delta delta;
            controller::NNInterface* nninterface;
    };
}

#endif
