#include "JumpController.h"

/* for namespace */
namespace controller
{
    /* for class Delta */
    /* private */

    /* public */
    void Delta::InverseKinematics(struct leg_data& leg)
    {
        double tempm,tempA,tempB,tempC,tempD;
        tempm = leg.endp_tar[0] * leg.endp_tar[0]
              + leg.endp_tar[1] * leg.endp_tar[1]
              + leg.endp_tar[2] * leg.endp_tar[2]
              + leg.rdif * leg.rdif
              + leg.uppleglen * leg.uppleglen
              - leg.lowleglen * leg.lowleglen;
        
        tempA = (tempm - 2 * leg.endp_tar[0] * leg.rdif)
              / (2 * leg.uppleglen)
              - (leg.rdif - leg.endp_tar[0]);
        tempB = 2 * leg.endp_tar[2];
        tempC = (tempm - 2 * leg.endp_tar[0] * leg.rdif)
              / (2 * leg.uppleglen)
              + (leg.rdif - leg.endp_tar[0]);
        tempD = sqrt(tempB * tempB - 4 * tempA * tempC);
        leg.joint_data[0].pos_tar = 2 * atan((tempB - tempD)/(2.0 * tempA));

        tempA = (tempm + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2 * leg.rdif
              - (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]);
        tempB = 4 * leg.endp_tar[2];
        tempC = (tempm + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2 * leg.rdif
              + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]);
        tempD = sqrt(tempB * tempB - 4 * tempA * tempC);
        leg.joint_data[1].pos_tar = 2 * atan((tempB - tempD)/(2.0 * tempA));

        tempA = (tempm + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2 * leg.rdif
              - (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]);
        tempB = 4 * leg.endp_tar[2];
        tempC = (tempm + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2 * leg.rdif
              + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]);
        tempD = sqrt(tempB * tempB - 4 * tempA * tempC);
        leg.joint_data[2].pos_tar = 2 * atan((tempB - tempD)/(2.0 * tempA));
    }

    void Delta::ForwardKinematics(struct leg_data& leg)
    {
        double a,b,c,p,s,lde,ld2f,lfe,lep;
        double d1[3],d2[3],d3[3],f[3],op[3];
        double d1d2[3],d2d3[3],d1d3[3],d2f[3],fe[3],ep[3];

        d1[0] = leg.rdif + leg.uppleglen * cos(leg.joint_data[0].pos);
        d1[1] = 0.0;
        d1[2] = leg.uppleglen * sin(leg.joint_data[0].pos);
        d2[0] = -1 / 2 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[1].pos));
        d2[1] = sqr3 / 2 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[1].pos));
        d2[2] = leg.uppleglen * sin(leg.joint_data[1].pos);
        d3[0] = -1 / 2 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[2].pos));
        d3[1] = -sqr3 / 2 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[2].pos));
        d3[2] = leg.uppleglen * sin(leg.joint_data[2].pos);
        VectorSub3(d2,d1,d1d2);
        VectorSub3(d3,d2,d2d3);
        VectorSub3(d3,d1,d1d3);
        
        f[0] = (d2[0] + d3[0]) / 2.0;
        f[1] = (d2[1] + d3[1]) / 2.0;
        f[2] = (d2[2] + d3[2]) / 2.0;
        VectorSub3(f,d2,d2f);
        ld2f = Norm3(d2f);

        a = Norm3(d1d2);
        b = Norm3(d2d3);
        c = Norm3(d1d3);
        p = (a + b + c) / 2.0;
        s = sqrt(p * (p - a) * (p - b) * (p - c));
        lde = a * b * c / 4.0 / s;

        CrossProduct3(d1d2,d1d3,ep);
        CrossProduct3(ep,d2d3,fe);
        lfe = Norm3(fe);
        fe[0]/= lfe; fe[1]/= lfe; fe[2]/= lfe;
        lfe = sqrt(lde * lde - ld2f * ld2f);
        fe[0]*= lfe; fe[1]*= lfe; fe[2]*= lfe;

        lep = Norm3(ep);
        ep[0]/= lep; ep[1]/= lep; ep[2]/= lep;
        lep = sqrt(leg.lowleglen * leg.lowleglen - lde * lde);
        ep[0]*= lep; ep[1]*= lep; ep[2]*= lep;

        for(int i = 0;i < 3;i++)
        {
            op[i] = f[i] + fe[i] + ep[i];
            leg.endp[i] = op[i];
        }
    }

    /* private */
    void Delta::VectorSub3(double* arr_front,double* arr_back,double* arr_out)
    {
        arr_out[0] = arr_front[0] - arr_back[0];
        arr_out[1] = arr_front[1] - arr_back[1];
        arr_out[2] = arr_front[2] - arr_back[2];
    }

    void Delta::CrossProduct3(double* arr_front,double* arr_back,double* arr_out)
    {
        arr_out[0] = arr_front[1] * arr_back[2] - arr_front[2] * arr_back[1];
        arr_out[1] = arr_front[2] * arr_back[0] - arr_front[0] * arr_back[2];
        arr_out[2] = arr_front[0] * arr_back[1] - arr_front[1] * arr_back[0];
    }

    double Delta::Norm3(double* arr)
    {
        return sqrt(arr[0]*arr[0] + arr[1]*arr[1] + arr[2]*arr[2]);
    }

    void JumpController::Get()
    {
        delta.ForwardKinematics(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.endp[i]<<std::endl;
            body.leg.endp_tar[i] = body.leg.endp[i];
        }
        delta.InverseKinematics(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.joint_data[i].pos_tar<<std::endl;
            body.leg.joint_data[i].pos = body.leg.joint_data[i].pos_tar;
        }
        delta.ForwardKinematics(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.endp[i]<<std::endl;
        }
    }
}


/* for node */
void InitJoint(controller::motor_data* joint_data,uint16_t joint_num)
{
    for(int i = 1;i <= joint_num;i++)
    {
        joint_data[i - 1].ID = i;
    }
}

void motorCallback(const can::motor_data::ConstPtr& motor_msg,controller::motor_data* joint_data)
{
    ROS_INFO("I heard");
    printf("ID:%d\n",(*motor_msg).id);
    printf("pos:%lf  vel:%lf\n",(*motor_msg).pos,(*motor_msg).vel);
    printf("tor:%lf  tem:%lf\n",(*motor_msg).tor,(*motor_msg).tem);
    joint_data[(*motor_msg).id - 1].pos = (*motor_msg).pos;
    joint_data[(*motor_msg).id - 1].vel = (*motor_msg).vel;
    joint_data[(*motor_msg).id - 1].tor = (*motor_msg).tor;
    joint_data[(*motor_msg).id - 1].tem = (*motor_msg).tem;
}

int main(int argc, char **argv)
{
    /* variables */
    int countl = 0, loop_hz = 100;
    const uint16_t jointnum = 3;
    controller::motor_data joint[jointnum];

    /* ros */
    ros::init(argc, argv, "JumpController");

    ros::NodeHandle nh;

    ros::Publisher motor_pub = 
    nh.advertise<can::motor_data>("cybergear_cmds", jointnum);

    ros::Subscriber motor_sub = 
    nh.subscribe<can::motor_data>("cybergear_msgs", 1000, boost::bind(&motorCallback,_1,joint));

    ros::Rate loop_rate(loop_hz);

    /* Initial */
    InitJoint(joint,jointnum);
    controller::JumpController jc(62.5,40,110,250,joint,jointnum);
    can::motor_data motor_cmd;

    /* loop */
    while (ros::ok())
    {
        ros::spinOnce();
        
        jc.Get();

        for(int i = 0;i < 3;i++)
        {
            motor_cmd.id = i + 1;
            motor_cmd.pos_tar = 0.0;
            motor_cmd.vel_tar = 0.0;
            motor_cmd.tor_tar = 0.0;
            motor_cmd.kp = 0.0;
            motor_cmd.kd = 0.0;
            motor_pub.publish(motor_cmd);
        }

        countl++;
        if(countl == loop_hz)
        {
            countl = 0;
            ROS_INFO("One Loop");
        }

        loop_rate.sleep();
    }

    return 0;
}