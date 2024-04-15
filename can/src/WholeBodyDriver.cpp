#include "WholeBodyDriver.h"

/* for namespace */
namespace wholebodydriver
{
    /* for class CyberGearDriver */
    /* private */
    void CyberGearDriver::Encode(uint16_t id,
                                 msg_type type,
                                 can_frame &frame)
    {
        frame.can_id = ((uint32_t)type) << 24;
        frame.can_id |= ((uint32_t)datainid) << 8;
        frame.can_id |= ((uint32_t)id);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        frame.data[0] = (uint8_t)(data8B[0] >> 8);
        frame.data[1] = (uint8_t)(data8B[0] & 0xff);
        frame.data[2] = (uint8_t)(data8B[1] >> 8);
        frame.data[3] = (uint8_t)(data8B[1] & 0xff);
        frame.data[4] = (uint8_t)(data8B[2] >> 8);
        frame.data[5] = (uint8_t)(data8B[2] & 0xff);
        frame.data[6] = (uint8_t)(data8B[3] >> 8);
        frame.data[7] = (uint8_t)(data8B[3] & 0xff);
    }

    void CyberGearDriver::Decode(can_frame &frame)
    {
        if (frame.can_id & CAN_EFF_FLAG)
        {
            frame.can_id &= CAN_EFF_MASK;
        }
        fbtype = (uint8_t)((frame.can_id << 3) >> 27);
        datainid = (uint16_t)((frame.can_id << 8) >> 16);
        fbid = (uint16_t)(frame.can_id & 0xff);
        data8B[0] = ((uint16_t)frame.data[0] << 8) | (uint16_t)frame.data[1];
        data8B[1] = ((uint16_t)frame.data[2] << 8) | (uint16_t)frame.data[3];
        data8B[2] = ((uint16_t)frame.data[4] << 8) | (uint16_t)frame.data[5];
        data8B[3] = ((uint16_t)frame.data[6] << 8) | (uint16_t)frame.data[7];
    }
    /* public */
    void CyberGearDriver::AskID(struct can_frame& frame,uint16_t id)
    {
        datainid = masterid;
        data8B[0] = 0;
        data8B[1] = 0;
        data8B[2] = 0;
        data8B[3] = 0;
        Encode(id,askID,frame);
    }

    void CyberGearDriver::MixControl(struct can_frame& frame,
                                    wholebodydriver::motor_data& motor_state)
    {
        datainid = (uint16_t)((motor_state.tor_tar + 12.0) / 24.0 * 65535.0);
        data8B[0] = (uint16_t)((motor_state.pos_tar + 4 * M_PI) / (8 * M_PI) * 65535.0);
        data8B[1] = (uint16_t)((motor_state.vel_tar + 30.0) / 60.0 * 65535.0);
        data8B[2] = (uint16_t)(motor_state.kp / 500.0 * 65535.0);
        data8B[3] = (uint16_t)(motor_state.kd / 5.0 * 65535.0);
        Encode(motor_state.ID,mixContorl,frame);
    }

    void CyberGearDriver::Enable(struct can_frame& frame,uint16_t id)
    {
        datainid = masterid;
        data8B[0] = 0;
        data8B[1] = 0;
        data8B[2] = 0;
        data8B[3] = 0;
        Encode(id,enableMotor,frame);
    }

    void CyberGearDriver::Disable(struct can_frame& frame,uint16_t id)
    {
        datainid = masterid;
        data8B[0] = 0;
        data8B[1] = 0;
        data8B[2] = 0;
        data8B[3] = 0;
        Encode(id,disableMotor,frame);
    }

    void CyberGearDriver::SetZero(struct can_frame& frame,uint16_t id)
    {
        datainid = masterid;
        data8B[0] = 0x0100;
        data8B[1] = 0;
        data8B[2] = 0;
        data8B[3] = 0;
        Encode(id,setZero,frame);
    }

    void CyberGearDriver::SetCANID(struct can_frame& frame,uint16_t id)
    {
        datainid = 0;
        datainid |= (0x01<<8);
        data8B[0] = 0;
        data8B[1] = 0;
        data8B[2] = 0;
        data8B[3] = 0;
        Encode(id,setCANID,frame);
    }

    void CyberGearDriver::SetSingleParam(struct can_frame& frame,uint16_t id)
    {

    }
    
    void CyberGearDriver::GetID(struct can_frame& frame)
    {

    }

    int CyberGearDriver::GetFeedback(struct can_frame& frame,
                                    wholebodydriver::motor_data& motor_state)
    {
        Decode(frame);
        if(fbtype == 2)
        {
            motor_state.ID = (datainid & 0xff);
            motor_state.pos = ((double)data8B[0] - 32767.0)/32768.0*4.0*M_PI;
            motor_state.vel = ((double)data8B[1] - 32767.0)/32768.0*30.0;
            motor_state.tor = ((double)data8B[2] - 32767.0)/32768.0*12.0;
            motor_state.tem = (double)data8B[3]/10.0;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    int CyberGearDriver::GetSingleParam(struct can_frame& frame)
    {
        Decode(frame);
        if(fbtype == 17)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    void CyberGearDriver::GetErrorState(struct can_frame& frame)
    {

    }

    /* for class WholeBodyDriver */
    /* private */
    int WholeBodyDriver::InitCan()
    {
        struct ifreq ifr0;
        int setflag = 0, ret0 = 0;
        std::string n0 = "can0";
        const char *name0 = n0.c_str();
        if ((s0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("Error while opening socket0");
        }
        strcpy(ifr0.ifr_name, name0);
        ioctl(s0, SIOCGIFINDEX, &ifr0);
        addr_0.can_family = AF_CAN;
        addr_0.can_ifindex = ifr0.ifr_ifindex;
        if (bind(s0, (struct sockaddr *)&addr_0, sizeof(addr_0)) < 0)
        {
            perror("Error in socket0 bind");
        }

        setflag = setflag | O_NONBLOCK;
        ret0 = fcntl(s0, F_SETFL, setflag);
        fcntl(s0, F_GETFL, 0);

        can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
        ret0 = setsockopt(s0, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
        if (ret0 != 0)
            printf("setsockopt0 fail\n");
        else
            printf("CAN Init Done\n");

        return ret0;
    }

    int WholeBodyDriver::SendCan(int s)
    {
        return write(s, &frame0_s, can_size);
    }

    int WholeBodyDriver::ReadCan(int s)
    {
        return read(s, &frame0_buf, can_size);
    }

    void WholeBodyDriver::CleanUp()
    {
        for(int i = 0;i < joint_num;i++)
        {
            cybergear.Disable(frame0_s,joint_data[i].ID);
            SendCan(s0);
        }
    }
    /* public */
    void WholeBodyDriver::Init()
    {
        InitCan();
        for(int i = 0;i < joint_num;i++)
        {
            cybergear.Enable(frame0_s,joint_data[i].ID);
            SendCan(s0);
            usleep(2000);
            while(1)
            {
                if(ReadCan(s0) == -1)
                {
                    printf("Enable motor%u end\n",joint_data[i].ID);
                    break;
                }
                printf("Enable motor%u success\n",joint_data[i].ID);
                memcpy(&frame0_r, &frame0_buf, sizeof(frame0_buf));
            }
            cybergear.GetFeedback(frame0_r,rx_data);
            for(int j = 0;j < joint_num;j++)
                if(joint_data[j].ID == rx_data.ID)
                {
                    joint_data[j].pos = rx_data.pos;
                    joint_data[j].vel = rx_data.vel;
                    joint_data[j].tor = rx_data.tor;
                    joint_data[j].tem = rx_data.tem;
                    printf("ID:%d\n",joint_data[j].ID);
                    printf("pos:%lf  vel:%lf\n",joint_data[j].pos,joint_data[j].vel);
                    printf("tor:%lf  tem:%lf\n",joint_data[j].tor,joint_data[j].tem);
                }

            // usleep(2000);
            // cybergear.SetZero(frame0_s,joint_data[i].ID);
            // SendCan(s0);
            // usleep(2000);
            // while(1)
            // {
            //     if(ReadCan(s0) == -1)
            //     {
            //         printf("Setzero motor%u end\n",joint_data[i].ID);
            //         break;
            //     }
            //     printf("Setzero motor%u success\n",joint_data[i].ID);
            //     memcpy(&frame0_r, &frame0_buf, sizeof(frame0_buf));
            // }
            // cybergear.GetFeedback(frame0_r,rx_data);
            // for(int j = 0;j < joint_num;j++)
            //     if(joint_data[j].ID == rx_data.ID)
            //     {
            //         joint_data[j].pos = rx_data.pos;
            //         joint_data[j].vel = rx_data.vel;
            //         joint_data[j].tor = rx_data.tor;
            //         joint_data[j].tem = rx_data.tem;
            //         printf("ID:%d\n",joint_data[j].ID);
            //         printf("pos:%lf  vel:%lf\n",joint_data[j].pos,joint_data[j].vel);
            //         printf("tor:%lf  tem:%lf\n",joint_data[j].tor,joint_data[j].tem);
            //     }
        }
    }

    void WholeBodyDriver::SendReadOnce()
    {
        for(int i = 0;i < joint_num;i++)
        {
            cybergear.MixControl(frame0_s,joint_data[i]);
            SendCan(s0);

            ReadCan(s0);
            cybergear.GetFeedback(frame0_buf,rx_data);

            for(int j = 0;j < joint_num;j++)
                if(joint_data[j].ID == rx_data.ID)
                {
                    joint_data[j].pos = rx_data.pos;
                    joint_data[j].vel = rx_data.vel;
                    joint_data[j].tor = rx_data.tor;
                    joint_data[j].tem = rx_data.tem;
                    printf("ID:%d\n",joint_data[j].ID);
                    printf("pos:%lf  vel:%lf\n",joint_data[j].pos,joint_data[j].vel);
                    printf("tor:%lf  tem:%lf\n",joint_data[j].tor,joint_data[j].tem);
                    com_success[j]++;
                }
        }
    }
}

/* for node */
void InitJoint(wholebodydriver::motor_data* joint_data,uint16_t joint_num)
{
    for(int i = 1;i <= joint_num;i++)
    {
        joint_data[i - 1].ID = i;
    }
}

void cybergearCallback(const can::motor_data::ConstPtr& motor_cmd,wholebodydriver::motor_data* joint_data)
{
    ROS_INFO("I heard");
    joint_data[(*motor_cmd).id - 1].pos_tar = (*motor_cmd).pos_tar;
    joint_data[(*motor_cmd).id - 1].vel_tar = (*motor_cmd).vel_tar;
    joint_data[(*motor_cmd).id - 1].tor_tar = (*motor_cmd).tor_tar;
    joint_data[(*motor_cmd).id - 1].kp = (*motor_cmd).kp;
    joint_data[(*motor_cmd).id - 1].kd = (*motor_cmd).kd;
}

int main(int argc, char **argv)
{
    /* variables */
    //int counth[3] = {0};
    int countl = 0,loop_hz = 500;
    double com_rate;
    const uint16_t jointnum = 3;
    int communication_success[jointnum];
    wholebodydriver::motor_data joint[jointnum];
    
    /* ros */
    ros::init(argc, argv, "WholeBodyDriver");

    ros::NodeHandle nh;

    ros::Publisher cybergear_pub = 
    nh.advertise<can::motor_data>("cybergear_msgs", jointnum);

    ros::Subscriber cybergear_sub = 
    nh.subscribe<can::motor_data>("cybergear_cmds", 1000, boost::bind(&cybergearCallback,_1,joint));

    ros::Rate loop_rate(loop_hz);

    /* Initial */
    InitJoint(joint,jointnum);
    wholebodydriver::WholeBodyDriver quata(joint,jointnum,communication_success);
    quata.Init();

    can::motor_data motor_msg;

    while (ros::ok())
    {
        ros::spinOnce();
        
        quata.SendReadOnce();

        for(int i = 0;i < jointnum;i++)
        {
            motor_msg.id = joint[i].ID;
            motor_msg.pos = joint[i].pos;
            motor_msg.vel = joint[i].vel;
            motor_msg.tor = joint[i].tor;
            motor_msg.tem = joint[i].tem;
            cybergear_pub.publish(motor_msg);
        }

        countl++;
        if(countl == loop_hz)
        {
            countl = 0;
            for(int i = 0;i < jointnum;i++)
            {
                com_rate = (double)communication_success[i] * 100.0 / loop_hz;
                communication_success[i] = 0;
                printf("motor%d:%lf%%\n",i + 1,com_rate);
            }
            ROS_INFO("One Loop");
        }

        loop_rate.sleep();
    }

    return 0;
}
