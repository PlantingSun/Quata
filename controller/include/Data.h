#ifndef DATA_H
#define DATA_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

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
        double endp[3],endv[3],endf[3],endp_wd[3];
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

        body_state state, last_state;
        double raw_ang_vel[3],ang_vel[3];
        double orient[4],rot_mat[3][3],rerot_mat[3][3],pitch,roll,yaw;
        double pos[3],vel[3],raw_vel[3],acc[3],raw_acc[3];
    };

    struct user_data{
        double pos[3],vel[3];
        double pitch,roll,yaw;
    };
}

#endif
