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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h>

#include "can/motor_data.h"

namespace controller{
    struct motor_data{
        uint16_t ID;
        double pos,vel,tor,tem;
        double pos_tar,vel_tar,tor_tar,kp,kd;
    };

    struct leg_data{
        motor_data* joint_data;
        
        double endp[3],endv[3],endf[3];
        double endp_tar[3],endv_tar[3],endf_tar[3];
        double biospos[3];
        double Jacob[3][3],reJacob[3][3];

        double rbody,rfoot,rdif;
        double uppleglen,lowleglen;
    };

    enum body_state{
            stateFlying = 0,
            stateLanding = 1,
            };

    struct body_data{
        leg_data leg;

        body_state state;
        double pitch,roll,yaw;
        double pos[3],vel[3];
    };

    /* Delta mechanism */
    class Delta{
        public:
            void InverseKinematics(struct leg_data& leg);
            void ForwardKinematicsP(struct leg_data& leg);
            void ForwardKinematicsVF(struct leg_data& leg);
            void CalJacob(struct leg_data& leg);
            void Statics(struct leg_data& leg);
        private:
            void VectorSub3(double* arr_front,double* arr_back,double* arr_out);
            void CrossProduct3(double* arr_front,double* arr_back,double* arr_out);
            void InverseMatrix33(double (*mat_in)[3],double (*mat_out)[3]);
            double Norm3(double* arr);
            const double sqr3 = 1.73205081;
            const double pi2_3 = M_PI * 2.0 / 3.0;
    };

    class JumpController{
        public:
            JumpController(double rbody_,double rfoot_,double uppleglen_,double lowleglen_,
                           controller::motor_data* joint_data_,uint16_t joint_num_,
                           double joint_kp_,double joint_kd_)
            {
                joint_num = joint_num_;
                body.leg.joint_data = joint_data_;
                body.leg.rbody = rbody_;
                body.leg.rfoot = rfoot_;
                body.leg.rdif = rbody_ - rfoot_;
                body.leg.uppleglen = uppleglen_;
                body.leg.lowleglen = lowleglen_;
                
                for(int i = 0;i < joint_num;i++)
                {
                    body.leg.joint_data[i].pos_tar = 0.0;
                    body.leg.joint_data[i].vel_tar = 0.0;
                    body.leg.joint_data[i].tor_tar = 0.0;
                    body.leg.joint_data[i].kp = joint_kp_;
                    body.leg.joint_data[i].kd = joint_kd_;
                }
            }
            void Get();
            void Init(double margin_);
            void Controller();
        private:
            void UpdateParam();
            void JudgeState();
            void SetFlyingAngle();
            void SetLandingForce();
            double margin;
            uint16_t joint_num,first_minimum = 0;
            controller::body_data body;
            controller::Delta delta;
    };
}

#endif