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
    //use data from body.leg.endp_tar[] and output to body.leg.joint_data[].pos_tar
    void Delta::InverseKinematics(struct leg_data& leg)
    {
        double tempm,tempA,tempB,tempC,tempD;
        tempm = leg.endp_tar[0] * leg.endp_tar[0]
              + leg.endp_tar[1] * leg.endp_tar[1]
              + leg.endp_tar[2] * leg.endp_tar[2]
              + leg.rdif * leg.rdif
              + leg.uppleglen * leg.uppleglen
              - leg.lowleglen * leg.lowleglen;
        
        tempA = (tempm - 2.0 * leg.endp_tar[0] * leg.rdif)
              / (2.0 * leg.uppleglen)
              - (leg.rdif - leg.endp_tar[0]);
        tempB = - 2.0 * leg.endp_tar[2];
        tempC = (tempm - 2.0 * leg.endp_tar[0] * leg.rdif)
              / (2.0 * leg.uppleglen)
              + (leg.rdif - leg.endp_tar[0]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        leg.joint_data[0].pos_tar = 2.0 * atan((- tempB - tempD)/(2.0 * tempA));

        tempA = (tempm + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2.0 * leg.rdif
              - (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]);
        tempB = - 4.0 * leg.endp_tar[2];
        tempC = (tempm + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2.0 * leg.rdif
              + (leg.endp_tar[0] - sqr3 * leg.endp_tar[1]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        leg.joint_data[1].pos_tar = 2.0 * atan((- tempB - tempD)/(2.0 * tempA));

        tempA = (tempm + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2.0 * leg.rdif
              - (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]);
        tempB = - 4.0 * leg.endp_tar[2];
        tempC = (tempm + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2.0 * leg.rdif
              + (leg.endp_tar[0] + sqr3 * leg.endp_tar[1]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        leg.joint_data[2].pos_tar = 2.0 * atan((- tempB - tempD)/(2.0 * tempA));
    }

    //use data from leg.joint_data[].pos and output to leg.endp
    void Delta::ForwardKinematics(struct leg_data& leg)
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

    void Delta::Statics(struct leg_data& leg)
    {
        double tempm,tempA,tempB,tempC,tempD,tempE;
        double tempmd[3],tempAd[3],tempBd[3],tempCd[3];
        double tempDd,tempEd;
        double tempJ[3][3];

        tempm = leg.endp[0] * leg.endp[0]
              + leg.endp[1] * leg.endp[1]
              + leg.endp[2] * leg.endp[2]
              + leg.rdif * leg.rdif
              + leg.uppleglen * leg.uppleglen
              - leg.lowleglen * leg.lowleglen;
        
        /*theta1 */
        tempA = (tempm - 2.0 * leg.endp[0] * leg.rdif)
              / (2.0 * leg.uppleglen)
              - (leg.rdif - leg.endp[0]);
        tempB = - 2.0 * leg.endp[2];
        tempC = (tempm - 2.0 * leg.endp[0] * leg.rdif)
              / (2.0 * leg.uppleglen)
              + (leg.rdif - leg.endp[0]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        tempE = (- tempB - tempD)/(2.0 * tempA);
        /* theta1_x */
        tempmd[0] = 2.0 * leg.endp[0];
        tempAd[0] = (tempmd[0] - 2.0 * leg.rdif) / (2.0 * leg.uppleglen) + 1.0;
        tempBd[0] = 0.0;
        tempCd[0] = (tempmd[0] - 2.0 * leg.rdif) / (2.0 * leg.uppleglen) - 1.0;
        /* theta1_y */
        tempmd[1] = 2.0 * leg.endp[1];
        tempAd[1] = tempmd[1] / (2.0 * leg.uppleglen);
        tempBd[1] = 0.0;
        tempCd[1] = tempmd[1] / (2.0 * leg.uppleglen);
        /* theta1_z */
        tempmd[2] = 2.0 * leg.endp[2];
        tempAd[2] = tempmd[2] / (2.0 * leg.uppleglen);
        tempBd[2] = - 2.0;
        tempCd[2] = tempmd[2] / (2.0 * leg.uppleglen);
        for(int i = 0;i < 3;i++)
        {
            tempDd = (2.0 * tempB * tempBd[i]
                   - 4.0 * tempA * tempCd[i] - 4.0 * tempAd[i] * tempC)
                   / tempD;
            tempEd = ((- tempBd[i] - tempDd) * 2.0 * tempA
                   - (- tempB - tempD) * 2.0 * tempAd[i])
                   / (4.0 * tempA * tempA);
            tempJ[0][i] = 2.0 * tempEd / (tempE * tempE + 1.0);
        }
        
        /* theta2 */
        tempA = (tempm + (leg.endp[0] - sqr3 * leg.endp[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2.0 * leg.rdif
              - (leg.endp[0] - sqr3 * leg.endp[1]);
        tempB = - 4.0 * leg.endp[2];
        tempC = (tempm + (leg.endp[0] - sqr3 * leg.endp[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2.0 * leg.rdif
              + (leg.endp[0] - sqr3 * leg.endp[1]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        tempE = (- tempB - tempD)/(2.0 * tempA);
        /* theta2_x */
        tempmd[0] = 2.0 * leg.endp[0];
        tempAd[0] = (tempmd[0] + leg.rdif) / leg.uppleglen - 1.0;
        tempBd[0] = 0.0;
        tempCd[0] = (tempmd[0] + leg.rdif) / leg.uppleglen + 1.0;
        /* theta2_y */
        tempmd[1] = 2.0 * leg.endp[1];
        tempAd[1] = (tempmd[1] - sqr3 * leg.rdif) / leg.uppleglen + sqr3;
        tempBd[1] = 0.0;
        tempCd[1] = (tempmd[1] - sqr3 * leg.rdif) / leg.uppleglen - sqr3;
        /* theta2_z */
        tempmd[2] = 2.0 * leg.endp[2];
        tempAd[2] = tempmd[2] / leg.uppleglen;
        tempBd[2] = - 4.0;
        tempCd[2] = tempmd[2] / leg.uppleglen;
        for(int i = 0;i < 3;i++)
        {
            tempDd = (2.0 * tempB * tempBd[i]
                   - 4.0 * tempA * tempCd[i] - 4.0 * tempAd[i] * tempC)
                   / tempD;
            tempEd = ((- tempBd[i] - tempDd) * 2.0 * tempA
                   - (- tempB - tempD) * 2.0 * tempAd[i])
                   / (4.0 * tempA * tempA);
            tempJ[1][i] = 2.0 * tempEd / (tempE * tempE + 1.0);
        }

        /* theta3 */
        tempA = (tempm + (leg.endp[0] + sqr3 * leg.endp[1]) * leg.rdif)
              / (leg.uppleglen)
              - 2.0 * leg.rdif
              - (leg.endp[0] + sqr3 * leg.endp[1]);
        tempB = - 4.0 * leg.endp[2];
        tempC = (tempm + (leg.endp[0] + sqr3 * leg.endp[1]) * leg.rdif)
              / (leg.uppleglen)
              + 2.0 * leg.rdif
              + (leg.endp[0] + sqr3 * leg.endp[1]);
        tempD = sqrt(tempB * tempB - 4.0 * tempA * tempC);
        tempE = (- tempB - tempD)/(2.0 * tempA);
        /* theta3_x */
        tempmd[0] = 2.0 * leg.endp[0];
        tempAd[0] = (tempmd[0] + leg.rdif) / leg.uppleglen - 1.0;
        tempBd[0] = 0.0;
        tempCd[0] = (tempmd[0] + leg.rdif) / leg.uppleglen + 1.0;
        /* theta3_y */
        tempmd[1] = 2.0 * leg.endp[1];
        tempAd[1] = (tempmd[1] + sqr3 * leg.rdif) / leg.uppleglen - sqr3;
        tempBd[1] = 0.0;
        tempCd[1] = (tempmd[1] + sqr3 * leg.rdif) / leg.uppleglen + sqr3;
        /* theta3_z */
        tempmd[2] = 2.0 * leg.endp[2];
        tempAd[2] = tempmd[2] / leg.uppleglen;
        tempBd[2] = - 4.0;
        tempCd[2] = tempmd[2] / leg.uppleglen;
        for(int i = 0;i < 3;i++)
        {
            tempDd = (2.0 * tempB * tempBd[i]
                   - 4.0 * tempA * tempCd[i] - 4.0 * tempAd[i] * tempC)
                   / tempD;
            tempEd = ((- tempBd[i] - tempDd) * 2.0 * tempA
                   - (- tempB - tempD) * 2.0 * tempAd[i])
                   / (4.0 * tempA * tempA);
            tempJ[2][i] = 2.0 * tempEd / (tempE * tempE + 1.0);
        }

        InverseMatrix33(tempJ,leg.Jacob);
        for(int i = 0;i < 3;i++)
        {
            for(int j = 0;j < 3;j++)
                printf("%lf ",tempJ[i][j]);
            printf("\n");
        }
        for(int i = 0;i < 3;i++)
        {
            leg.joint_data[i].tor_tar = 0.0;
            for(int j = 0;j < 3;j++)
            {
                printf("%lf ",leg.Jacob[i][j]);
                leg.joint_data[i].tor_tar+= leg.Jacob[j][i] * leg.endf_tar[j] / 1000.0;
            }
            printf("\n");
        }
    }

    //
    void JumpController::Get()
    {
        cout << "Get" << endl;
        delta.ForwardKinematics(body.leg);
        cout << "end position:" << endl;
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.endp[i]<<std::endl;
            // body.leg.endp_tar[i] = body.leg.endp[i];
        }
        body.leg.endf_tar[0] = 0.0;
        body.leg.endf_tar[1] = 0.0;
        body.leg.endf_tar[2] = 13.0;
        delta.Statics(body.leg);
        delta.InverseKinematics(body.leg);
<<<<<<< HEAD
        cout << "joint position:" << endl;
=======

>>>>>>> eee3ba511531791e417099d25c8b16f7abbee319
        for(int i = 0;i < joint_num;i++)
        {
            std::cout<<body.leg.joint_data[i].pos_tar<<
            " "<<body.leg.joint_data[i].tor_tar<<std::endl;
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

void ik(controller::JumpController jc)
{
    jc.delta.InverseKinematics(jc.body.leg);
    cout << "Target joint position:" << endl;
    for(int i = 0;i < jc.joint_num;i++)
    {
        std::cout<<jc.body.leg.joint_data[i].pos_tar<<std::endl;
        // body.leg.joint_data[i].pos = body.leg.joint_data[i].pos_tar;
    }
    printf("\n\n");
}



int main(int argc, char **argv)
{
    /* variables */
<<<<<<< HEAD
    int countl = 0, loop_hz = 10;
=======
    int countl = 0, loop_hz = 1;
>>>>>>> eee3ba511531791e417099d25c8b16f7abbee319
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
<<<<<<< HEAD
    controller::JumpController jc(62.5,40,110,250,joint,jointnum);
    can::motor_data motor_cmd[3];

    //test
    //fk: leg.joint_data[].pos and output to leg.endp
    //ik: body.leg.endp_tar[3] to body.leg.joint_data[].pos_tar
    

    jc.delta.InverseKinematics(jc.body.leg);
    cout << "joint position:" << endl;
    for(int i = 0;i < 3;i++)
    {
        std::cout<<jc.body.leg.joint_data[i].pos_tar<<std::endl;
        // body.leg.joint_data[i].pos = body.leg.joint_data[i].pos_tar;
    }
    // system("pause");
=======
    controller::JumpController jc(62.5,40.0,110.0,250.0,joint,jointnum);
    can::motor_data motor_cmd;
>>>>>>> eee3ba511531791e417099d25c8b16f7abbee319

    // cout << 111111 << endl;
    /* loop */
    double KP = 1, plus = 0.1;
    double x_start = -0.3, x_end = -0.31, duration = 5000,xcur = x_start;
    int x = 0;
    while (ros::ok())
    {   
        // if(x++ > 200)
        //     return 0;
        // if(xcur != x_end)
        //     xcur += (x_end - x_start) / duration;

        ros::spinOnce();
<<<<<<< HEAD
=======

        for(int i = 0;i < 3;i++)
        {
            joint[i].pos = 0.1 * (double)i;
        }

        jc.Get();
>>>>>>> eee3ba511531791e417099d25c8b16f7abbee319

        jc.body.leg.endp_tar[0] = 0.5;
        jc.body.leg.endp_tar[1] = 0;
        jc.body.leg.endp_tar[2] = 250;
        // cout << xcur << endl;
        // ik(jc);

        // jc.Get();

        // motor_cmd[0].pos_tar = 0.0;
        // motor_cmd[1].pos_tar = 1;
        // motor_cmd[2].pos_tar = 0;

        
        for(int i = 0;i < 3;i++)
        {
<<<<<<< HEAD
            motor_cmd[i].id = i+1;
            // motor_cmd[i].pos_tar = jc.body.leg.joint_data[i].pos_tar;
            motor_cmd[i].pos_tar = 1;
            motor_cmd[i].vel_tar = 0.0;
            motor_cmd[i].tor_tar = 0.0;
            motor_cmd[i].kp = 5;
            motor_cmd[i].kd = 0.1;
            motor_pub.publish(motor_cmd[i]);
=======
            motor_cmd.id = i + 1;
            motor_cmd.pos_tar = 0.0;
            motor_cmd.vel_tar = 0.0;
            motor_cmd.tor_tar = 0.0;
            motor_cmd.kp = 2.5;
            motor_cmd.kd = 0.0;
            motor_pub.publish(motor_cmd);
>>>>>>> eee3ba511531791e417099d25c8b16f7abbee319
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