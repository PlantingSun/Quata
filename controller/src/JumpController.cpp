#include "JumpController.h"

/* for namespace */
namespace controller
{
    /* for class Delta */
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

    void Delta::InverseMatrix33(double (*mat_in)[3],double (*mat_out)[3])
    {
        double tempdet = 0.0,tempadj;
        int tempi1,tempi2,tempj1,tempj2;

        for(int i = 0;i < 3;i++)
        {
            tempi1 = (i + 1) % 3;
            tempi2 = (i + 2) % 3;
            tempdet+= mat_in[0][i] * mat_in[1][tempi1] * mat_in[2][tempi2];
            tempdet-= mat_in[0][i] * mat_in[2][tempi1] * mat_in[1][tempi2];
        }
        printf("%lf\n",tempdet);
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
            {
                tempi1 = (i + 1) % 3; tempj1 = (j + 1) % 3;
                tempi2 = (i + 2) % 3; tempj2 = (j + 2) % 3;
                tempadj = mat_in[tempj1][tempi1] * mat_in[tempj2][tempi2]
                - mat_in[tempj1][tempi2] * mat_in[tempj2][tempi1];
                mat_out[i][j] = tempadj / tempdet;
            }
        
    }

    double Delta::Norm3(double* arr)
    {
        return sqrt(arr[0]*arr[0] + arr[1]*arr[1] + arr[2]*arr[2]);
    }

    /* public */
    void Delta::InverseKinematics(struct leg_data& leg)
    {
        double tempm,tempA,tempB,tempC,tempD,tempE,tempxy;
        tempm = leg.endp_tar[0] * leg.endp_tar[0]
              + leg.endp_tar[1] * leg.endp_tar[1]
              + leg.endp_tar[2] * leg.endp_tar[2]
              + leg.rdif * leg.rdif
              + leg.uppleglen * leg.uppleglen
              - leg.lowleglen * leg.lowleglen;

        for(int i = 0;i < 3;i++)
        {
            tempxy = cos(pi2_3 * i) * leg.endp_tar[0] + sin(pi2_3 * i) * leg.endp_tar[1];
            tempA = (tempm - 2.0 * leg.rdif * tempxy)
                  / (2.0 * leg.uppleglen)
                  - leg.rdif + tempxy;
            tempB = - 2.0 * leg.endp_tar[2];
            tempC = (tempm - 2.0 * leg.rdif * tempxy)
                  / (2.0 * leg.uppleglen)
                  + leg.rdif - tempxy;
            tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
            tempE = (- tempB - tempD)/(2.0 * tempA);
            leg.joint_data[i].pos_tar = 2.0 * atan(tempE);
        }
    }

    void Delta::ForwardKinematicsP(struct leg_data& leg)
    {
        double a,b,c,p,s,lde,ld2f,lfe,lep;
        double d1[3],d2[3],d3[3],f[3],op[3];
        double d1d2[3],d2d3[3],d1d3[3],d2f[3],fe[3],ep[3];

        d1[0] = leg.rdif + leg.uppleglen * cos(leg.joint_data[0].pos);
        d1[1] = 0.0;
        d1[2] = leg.uppleglen * sin(leg.joint_data[0].pos);
        d2[0] = -1.0 / 2.0 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[1].pos));
        d2[1] = sqr3 / 2.0 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[1].pos));
        d2[2] = leg.uppleglen * sin(leg.joint_data[1].pos);
        d3[0] = -1.0 / 2.0 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[2].pos));
        d3[1] = -sqr3 / 2.0 * (leg.rdif + leg.uppleglen * cos(leg.joint_data[2].pos));
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

    void Delta::ForwardKinematicsVF(struct leg_data& leg)
    {
        for(int i = 0;i < 3;i++)
        {
            leg.endv[i] = 0.0;
            leg.endf[i] = 0.0;
            for(int j = 0;j < 3;j++)
            {
                leg.endv[i]+= leg.Jacob[i][j] * leg.joint_data[j].vel;
                leg.endf[i]+= leg.reJacob[j][i] * leg.joint_data[j].tor;
            }
        }
    }

    void Delta::CalJacob(struct leg_data& leg)
    {
        double tempm,tempA,tempB,tempC,tempD,tempE,tempxy;
        double tempmd,tempxyd,tempAd,tempBd,tempCd;
        double tempDd,tempEd;

        tempm = leg.endp[0] * leg.endp[0]
              + leg.endp[1] * leg.endp[1]
              + leg.endp[2] * leg.endp[2]
              + leg.rdif * leg.rdif
              + leg.uppleglen * leg.uppleglen
              - leg.lowleglen * leg.lowleglen;

        for(int i = 0;i < 3;i++)
        {
            tempxy = cos(pi2_3 * i) * leg.endp[0] + sin(pi2_3 * i) * leg.endp[1];
            tempA = (tempm - 2.0 * leg.rdif * tempxy)
                  / (2.0 * leg.uppleglen)
                  - leg.rdif + tempxy;
            tempB = - 2.0 * leg.endp[2];
            tempC = (tempm - 2.0 * leg.rdif * tempxy)
                  / (2.0 * leg.uppleglen)
                  + leg.rdif - tempxy;
            tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
            tempE = (- tempB - tempD)/(2.0 * tempA);
            for(int j = 0;j < 3;j++)
            {
                tempmd = 2.0 * leg.endp[j];
                if(j < 2)
                {
                    tempxyd = cos((M_PI_2 * j) - pi2_3 * i);
                    tempAd = (tempmd - 2.0 * leg.rdif * tempxyd)
                           / (2.0 * leg.uppleglen) + tempxyd;
                    tempBd = 0.0;
                    tempCd = (tempmd - 2.0 * leg.rdif * tempxyd)
                           / (2.0 * leg.uppleglen) - tempxyd;
                }
                else
                {
                    tempAd = tempmd / (2.0 * leg.uppleglen);
                    tempBd = - 2.0;
                    tempCd = tempmd / (2.0 * leg.uppleglen);
                }
                tempDd = 0.5 / tempD
                       * (2.0 * tempB * tempBd
                       - 4.0 * tempA * tempCd - 4.0 * tempAd * tempC);
                tempEd = 1.0 / (4.0 * tempA * tempA)
                       * ((- tempBd - tempDd) * 2.0 * tempA
                       - (- tempB - tempD) * 2.0 * tempAd);
                leg.reJacob[i][j] = 2.0 * tempEd / (tempE * tempE + 1.0);
            }
        }
        InverseMatrix33(leg.reJacob,leg.Jacob);
    }

    void Delta::Statics(struct leg_data& leg)
    {
        for(int i = 0;i < 3;i++)
        {
            leg.joint_data[i].tor_tar = 0.0;
            for(int j = 0;j < 3;j++)
            {
                leg.joint_data[i].tor_tar+= leg.Jacob[j][i] * leg.endf_tar[j] / 1000.0;
            }
        }
    }

    void JumpController::Get()
    {
        delta.ForwardKinematicsP(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.endp[i]<<std::endl;
            body.leg.endp_tar[i] = body.leg.endp[i];
        }

        body.leg.endf_tar[0] = - 0.05 * (body.leg.endp[0] - 0.0);
        body.leg.endf_tar[1] = - 0.05 * (body.leg.endp[1] - 0.0);
        body.leg.endf_tar[2] = - 0.5 * (body.leg.endp[2] - 212.0) + 1.60;
        printf("%lf %lf %lf\n",body.leg.endf_tar[0],body.leg.endf_tar[1],body.leg.endf_tar[2]);
        delta.Statics(body.leg);

        delta.InverseKinematics(body.leg);

        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.joint_data[i].pos_tar<<
            " "<<body.leg.joint_data[i].tor_tar<<std::endl;
        }
    }

    void JumpController::UpdateParam()
    {
        delta.ForwardKinematicsP(body.leg);
        delta.CalJacob(body.leg);
        delta.ForwardKinematicsVF(body.leg);

        for(int i = 0;i < 3;i++)
        {
            printf("%lf ",body.leg.endp[i]);
        }
        printf("\n");
        /* IMU for Body */
    }

    void JumpController::JudgeState()
    {
        if(first_minimum)
        {
            if(body.pos[2] - body.leg.endp[2] > margin)
                body.state = stateFlying;
            else
                body.state = stateLanding;
        }
        else
        {
            body.state = stateLanding;
            if(body.vel[2] > 0)
            {
                first_minimum = 1;
                body.pos[2] = body.leg.endp[2];
            }
        }
    }

    void JumpController::SetFlyingAngle()
    {

    }

    void JumpController::SetLandingForce()
    {
        body.leg.endf_tar[0] = 1.0;
        body.leg.endf_tar[1] = 0.0;
        body.leg.endf_tar[2] = 0.0;
        delta.Statics(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            body.leg.joint_data[i].kp = 0.0;
        }
    }
    
    void JumpController::Init(double margin_)
    {
        margin = margin_;
    }

    void JumpController::Controller()
    {
        UpdateParam();
        // JudgeState();
        // if(body.state == stateFlying)
        // {
        //     SetFlyingAngle();
        //     SetLandingForce();
        // }
        // else
        // {
        //     SetLandingForce();
        // }

        SetLandingForce();
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
    nh.subscribe<can::motor_data>("cybergear_msgs", jointnum, boost::bind(&motorCallback,_1,joint));

    ros::Rate loop_rate(loop_hz);

    /* Initial */
    InitJoint(joint,jointnum);
    controller::JumpController jc(62.5,40.0,110.0,250.0,joint,jointnum,2.5,0.1);
    jc.Init(10);
    can::motor_data motor_cmd;
    

    /* loop */
    while (ros::ok())
    {
        ros::spinOnce();

        jc.Controller();

        for(int i = 0;i < 3;i++)
        {
            motor_cmd.id = i + 1;
            motor_cmd.pos_tar = joint[i].pos_tar;
            motor_cmd.vel_tar = joint[i].vel_tar;
            motor_cmd.tor_tar = joint[i].tor_tar;
            motor_cmd.kp = joint[i].kp;
            motor_cmd.kd = joint[i].kd;
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