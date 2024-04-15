#ifndef WHOLEBODYDRIVER_H
#define WHOLEBODYDRIVER_H

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

namespace wholebodydriver{
struct motor_data{
    uint16_t ID;
    double pos,vel,tor,tem;
    double pos_tar,vel_tar,tor_tar,kp,kd;
};

/* CyberGear Instruction Generator */
class CyberGearDriver{
public:
    void AskID(struct can_frame& frame,uint16_t id);
    void MixControl(struct can_frame& frame,
                    wholebodydriver::motor_data& motor_state);
    void Enable(struct can_frame& frame,uint16_t id);
    void Disable(struct can_frame& frame,uint16_t id);
    void SetZero(struct can_frame& frame,uint16_t id);
    void SetCANID(struct can_frame& frame,uint16_t id);
    void SetSingleParam(struct can_frame& frame,uint16_t id);
    
    void GetID(struct can_frame& frame);
    int GetFeedback(struct can_frame& frame,
                    wholebodydriver::motor_data& motor_state);
    int GetSingleParam(struct can_frame& frame);
    void GetErrorState(struct can_frame& frame);

private:
    enum msg_type{
    askID = 0,
    mixContorl = 1,
    enableMotor = 3,
    disableMotor = 4,
    setZero = 6,
    setCANID = 7,
    readParam = 17,
    writeParam = 18,

    motorFeedback = 2,
    errorFeedback = 21,
    };

    void Encode(uint16_t id,
                msg_type type,
                can_frame& frame);
    void Decode(can_frame& frame);

    uint8_t fbtype;
    uint16_t fbid;
    uint16_t masterid = 0;
    uint16_t datainid,data8B[4];
};

/* CAN Driver for Whole Body */
class WholeBodyDriver{
public:
    WholeBodyDriver(wholebodydriver::motor_data* joint_data_,uint16_t joint_num_,
                    int* com_success_)
    {
        joint_data = joint_data_;
        joint_num = joint_num_;
        com_success = com_success_;
    }
    void Init();
    void SendReadOnce();
    ~WholeBodyDriver()
    {
        CleanUp();
    }
private:
    int InitCan();
    int SendCan(int s);
    int ReadCan(int s);
    void CleanUp();

    uint16_t joint_num;
    int* com_success;
    wholebodydriver::motor_data* joint_data;
    wholebodydriver::motor_data rx_data,temp_cmd;
    wholebodydriver::CyberGearDriver cybergear;

    int s0;
    int nbytes;
    const int can_size = sizeof(struct can_frame);
    struct sockaddr_can addr_0;
    struct can_frame frame0_s,frame0_r,frame0_buf;
};

    
}

#endif