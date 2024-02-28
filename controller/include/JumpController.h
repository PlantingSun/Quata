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
#include "sensor_msgs/Imu.h"

#include <unistd.h>

#include "can/motor_data.h"

namespace controller{

    struct motor_data{
        uint16_t ID;
        double pos,vel,tor,tem;
        double pos_tar,vel_tar,tor_tar,kp,kd;
    };

    struct leg_data{
        int joint_num;
        motor_data* joint_data;
        
        double len;
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
        int leg_num;
        leg_data* leg;

        body_state state;
        double orient[4],rot_mat[3][3],pitch,roll,yaw;
        double pos[3],vel[3],acc[3],raw_acc[3];
    };

    /* Delta mechanism */
    class Delta{
        public:
            void InverseKinematics(struct leg_data& leg);
            void ForwardKinematicsP(struct leg_data& leg);
            void ForwardKinematicsVF(struct leg_data& leg);
            void CalJacob(struct leg_data& leg);
            void CalreJacob(struct leg_data& leg);
            void Statics(struct leg_data& leg);
        private:
            void VectorSub3(double* arr_front,double* arr_back,double* arr_out);
            void MatrixSub33(double (*mat_front)[3],double (*mat_back)[3],double (*mat_out)[3]);
            void CrossProduct3(double* arr_front,double* arr_back,double* arr_out);
            void InverseMatrix33(double (*mat_in)[3],double (*mat_out)[3]);
            double Norm3(double* arr);
            void Norm3d(double* arr,double (*arr_d)[3],double len,double* len_d);
            const double sqr3 = 1.73205081;
            const double pi2_3 = M_PI * 2.0 / 3.0;
    };

    /* controller */
    class JumpController{
        public:
            JumpController(double l_0_,double k_spring_,double margin_,
                           double joint_kp_,double joint_kd_,double ctrl_rate_)
            {
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
            void QuattoRotMat(controller::body_data& body);
            void QuattoEuler(controller::body_data& body);
            void UpdateParam(controller::body_data& body);
            void JudgeState(controller::body_data& body);
            void SetFlyingAngle(controller::body_data& body);
            void SetLandingForce(controller::body_data& body);
            double l_0,k_spring,margin;
            double joint_kp,joint_kd,ctrl_rate;
            int first_minimum;

            controller::Delta delta;
    };
}

#endif
