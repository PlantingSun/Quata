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
using namespace std;

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
        double Jacob[3][3];

        double rbody,rfoot,rdif;    //radius of body,foot,difference
        double uppleglen,lowleglen;
    };

    struct body_data{
        leg_data leg;

        double pitch,roll,yaw;
        double posx,posy,posz;
    };

    /* Delta mechanism */
    class Delta{
        public:
            void InverseKinematics(struct leg_data& leg);
            void ForwardKinematics(struct leg_data& leg);
            void Statics(struct leg_data& leg);
        private:
            void VectorSub3(double* arr_front,double* arr_back,double* arr_out);
            void CrossProduct3(double* arr_front,double* arr_back,double* arr_out);
            void InverseMatrix33(double (*mat_in)[3],double (*mat_out)[3]);
            double Norm3(double* arr);
            const double sqr3 = 1.73205081;
    };

    class JumpController{
        public:
            JumpController(double rbody_,double rfoot_,double uppleglen_,double lowleglen_,
                           controller::motor_data* joint_data_,uint16_t joint_num_)
            {
                joint_num = joint_num_;
                body.leg.joint_data = joint_data_;
                body.leg.rbody = rbody_;
                body.leg.rfoot = rfoot_;
                body.leg.rdif = rbody_ - rfoot_;
                body.leg.uppleglen = uppleglen_;
                body.leg.lowleglen = lowleglen_;
            }
            void Get();
            void Init();
        // private:

        uint16_t joint_num;
        controller::body_data body;
        controller::Delta delta;
    };
}

#endif