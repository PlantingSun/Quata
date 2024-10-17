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
#include "geometry_msgs/Point.h"

#include "Data.h"
#include "DeltaMechanism.h"
// #include "NNInterface.h"

namespace controller{
    /* controller */
    class JumpController{
        public:
            JumpController(double l_0_,double k_spring_,double margin_,double friction_,
                           double base_vel_kp_,double base_hei_kp_,double base_hei_kd_,double base_att_kp_,double base_att_kd_,
                           double joint_kp_,double joint_kd_,double ctrl_rate_)
            {
                l_0 = l_0_;
                k_spring = k_spring_;
                margin = margin_;
                friction = friction_;
                base_vel_kp = base_vel_kp_;
                base_hei_kp = base_hei_kp_;
                base_hei_kd = base_hei_kd_;
                base_att_kp = base_att_kp_;
                base_att_kd = base_att_kd_;
                joint_kp = joint_kp_;
                joint_kd = joint_kd_;
                ctrl_rate = ctrl_rate_;
            }
            void Controller(controller::body_data& body,controller::user_data& user);
        private:
            void QuattoRotMat(double* quat,double (*rot_mat)[3]);
            void QuattoEuler(double* quat,double& roll,double& pitch,double& yaw);
            void UpdateParam(controller::body_data& body);
            void JudgeState(controller::body_data& body,controller::user_data& user);
            void SetFlyingAngle(controller::body_data& body,controller::user_data& user);
            void SetLandingForce(controller::body_data& body,controller::user_data& user);
            double cmdpos[3],cmdpos_kp[3];
            double fe_wd[3],state_height,height_err,integer_height_err,max_torque;
            double base_hei_kp,base_hei_kd;
            double base_att_kp,base_att_kd,friction;
            double last_len;
            double stance_time,last_stance_time;
            double pe_wd[3],base_vel_kp;
            double l_0,k_spring,margin;
            double joint_kp,joint_kd,ctrl_rate;
            bool first_jump = 0;

            controller::Delta delta;
    };
}

#endif
