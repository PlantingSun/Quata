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

    void Delta::MatrixSub33(double (*mat_front)[3],double (*mat_back)[3],double (*mat_out)[3])
    {
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
                mat_out[i][j] = mat_front[i][j] - mat_back[i][j];
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

    void Delta::Norm3d(double* arr,double (*arr_d)[3],double len,double* len_d)
    {
        for(int i = 0;i < 3;i++)
        {
            len_d[i] = 0.0;
            for(int j = 0;j < 3;j++)
                len_d[i]+= arr[j] * arr_d[j][i];
            len_d[i]/= len;
        }
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
        leg.len = Norm3(leg.endp);
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
        double a,b,c,p,s,lde,ld2f,lfe,lfe0,lep,lep0;
        double ad[3],bd[3],cd[3],pd[3],sd[3],lded[3],ld2fd[3],lfed[3],lfe0d[3],lepd[3],lep0d[3];

        double d1[3],d2[3],d3[3],f[3],op[3];
        double d1d[3][3] = {0.0},d2d[3][3] = {0.0},d3d[3][3] = {0.0};
        double fd[3][3],opd[3][3];

        double d1d2[3],d2d3[3],d1d3[3],d2f[3],fe[3],fe0[3],ep[3],ep0[3];
        double d1d2d[3][3],d2d3d[3][3],d1d3d[3][3],d2fd[3][3],fed[3][3],fe0d[3][3],epd[3][3],ep0d[3][3];

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
        d1d[0][0] = - leg.uppleglen * sin(leg.joint_data[0].pos);
        d1d[2][0] = leg.uppleglen * cos(leg.joint_data[0].pos);
        d2d[0][1] = 1.0 / 2.0 * leg.uppleglen * sin(leg.joint_data[1].pos);
        d2d[1][1] = - sqr3 / 2.0 * leg.uppleglen * sin(leg.joint_data[1].pos);
        d2d[2][1] = leg.uppleglen * cos(leg.joint_data[1].pos);
        d3d[0][2] = 1.0 / 2.0 * leg.uppleglen * sin(leg.joint_data[2].pos);
        d3d[1][2] = sqr3 / 2.0 * leg.uppleglen * sin(leg.joint_data[2].pos);
        d3d[2][2] = leg.uppleglen * cos(leg.joint_data[2].pos);
        MatrixSub33(d2d,d1d,d1d2d);
        MatrixSub33(d3d,d2d,d2d3d);
        MatrixSub33(d3d,d1d,d1d3d);

        f[0] = (d2[0] + d3[0]) / 2.0;
        f[1] = (d2[1] + d3[1]) / 2.0;
        f[2] = (d2[2] + d3[2]) / 2.0;
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
                fd[i][j] = (d2d[i][j] + d3d[i][j]) / 2.0;

        VectorSub3(f,d2,d2f);
        MatrixSub33(fd,d2d,d2fd);

        ld2f = Norm3(d2f);
        Norm3d(d2f,d2fd,ld2f,ld2fd);

        a = Norm3(d1d2);
        b = Norm3(d2d3);
        c = Norm3(d1d3);
        Norm3d(d1d2,d1d2d,a,ad);
        Norm3d(d2d3,d2d3d,b,bd);
        Norm3d(d1d3,d1d3d,c,cd);

        p = (a + b + c) / 2.0;
        for(int i = 0;i < 3;i++)
            pd[i] = (ad[i] + bd[i] + cd[i]) / 2.0;

        double tempsum,tempsumd,dsda,dsdb,dsdc;
        tempsum = p * (p - a) * (p - b) * (p - c);
        s = sqrt(tempsum);
        tempsumd = 1 / p + 1 / (p - a) + 1 / (p - b) + 1 / (p - c);
        dsda = (tempsumd - 2.0 / (p - a)) * s / 4.0;
        dsdb = (tempsumd - 2.0 / (p - b)) * s / 4.0;
        dsdc = (tempsumd - 2.0 / (p - c)) * s / 4.0;
        for(int i = 0;i < 3;i++)
            sd[i] = dsda * ad[i] + dsdb * bd[i] + dsdc * cd[i];

        lde = a * b * c / 4.0 / s;
        for(int i = 0;i < 3;i++)
            lded[i] = ((ad[i] * b * c + a * bd[i] * c + a * b * cd[i]) * s
                    - a * b * c * sd[i]) / (4.0 * s * s);

        CrossProduct3(d1d2,d1d3,ep);
        CrossProduct3(d1d2,d1d3,ep0);
        CrossProduct3(ep,d2d3,fe);
        CrossProduct3(ep,d2d3,fe0);
        lfe0 = Norm3(fe0);
        double tempddd[3],tempcrossd[3],tempfed[3];
        for(int i = 0;i < 3;i++)
        {
            for(int j = 0;j < 3;j++)
                tempddd[j] = d1d2d[j][i];
            CrossProduct3(tempddd,d1d3,tempcrossd);
            CrossProduct3(tempcrossd,d2d3,tempfed);
            for(int j = 0;j < 3;j++)
                fe0d[j][i] = tempfed[j];

            for(int j = 0;j < 3;j++)
                tempddd[j] = d1d3d[j][i];
            CrossProduct3(d1d2,tempddd,tempcrossd);
            CrossProduct3(tempcrossd,d2d3,tempfed);
            for(int j = 0;j < 3;j++)
                fe0d[j][i]+= tempfed[j]; 

            for(int j = 0;j < 3;j++)
                tempddd[j] = d2d3d[j][i];
            CrossProduct3(d1d2,d1d3,tempcrossd);
            CrossProduct3(tempcrossd,tempddd,tempfed);
            for(int j = 0;j < 3;j++)
                fe0d[j][i]+= tempfed[j];
        }
        Norm3d(fe0,fe0d,lfe0,lfe0d);

        lfe = Norm3(fe);
        fe[0]/= lfe; fe[1]/= lfe; fe[2]/= lfe;
        lfe = sqrt(lde * lde - ld2f * ld2f);
        fe[0]*= lfe; fe[1]*= lfe; fe[2]*= lfe;
        for(int i = 0;i < 3;i++)
            lfed[i] = (lde * lded[i] - ld2f * ld2fd[i]) / lfe;
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
            {
                fed[i][j] = ((fe0d[i][j] * lfe + fe0[i] * lfed[j]) * lfe0
                          - fe0[i] * lfe * lfe0d[j]) / lfe0 / lfe0;
            }

        
        lep0 = Norm3(ep0);
        for(int i = 0;i < 3;i++)
        {
            for(int j = 0;j < 3;j++)
                tempddd[j] = d1d2d[j][i];
            CrossProduct3(tempddd,d1d3,tempcrossd);
            for(int j = 0;j < 3;j++)
                ep0d[j][i] = tempcrossd[j];

            for(int j = 0;j < 3;j++)
                tempddd[j] = d1d3d[j][i];
            CrossProduct3(d1d2,tempddd,tempcrossd);
            for(int j = 0;j < 3;j++)
                ep0d[j][i]+= tempcrossd[j];
        }
        Norm3d(ep0,ep0d,lep0,lep0d);

        lep = Norm3(ep);
        ep[0]/= lep; ep[1]/= lep; ep[2]/= lep;
        lep = sqrt(leg.lowleglen * leg.lowleglen - lde * lde);
        ep[0]*= lep; ep[1]*= lep; ep[2]*= lep;
        for(int i = 0;i < 3;i++)
            lepd[i] = - lde * lded[i] / lep;
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
            {
                epd[i][j] = ((ep0d[i][j] * lep + ep0[i] * lepd[j]) * lep0
                          - ep0[i] * lep * lep0d[j]) / lep0 / lep0;
            }

        for(int i = 0;i < 3;i++)
        {
            op[i] = f[i] + fe[i] + ep[i];
            leg.endp[i] = op[i];
            for(int j = 0;j < 3;j++)
            {
                opd[i][j] = fd[i][j] + fed[i][j] + epd[i][j];
                leg.Jacob[i][j] = opd[i][j];
            }
        }
        InverseMatrix33(leg.Jacob,leg.reJacob);
        leg.len = Norm3(leg.endp);
    }

    void Delta::CalreJacob(struct leg_data& leg)
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
        // InverseMatrix33(leg.reJacob,leg.Jacob);
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
        // delta.ForwardKinematicsP(body.leg);
        delta.CalJacob(body.leg);
        // delta.CalreJacob(body.leg);
        delta.ForwardKinematicsVF(body.leg);

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
            // if(body.vel[2] > 0)
            if(body.leg.endv[2] > 0)
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
        // for(int i = 0;i < joint_num;i++)
        // {
        //     body.leg.endf_tar[i] = - k_spring * (1 - l_0/body.leg.len) * body.leg.endp[i];
        //     if(body.leg.endv[2] > 0)
        //         body.leg.endf_tar[i]*= 5.0;
        // }
        body.leg.endf_tar[0] = 0.0;
        body.leg.endf_tar[1] = 0.0;
        body.leg.endf_tar[2] = 3.0;
        delta.Statics(body.leg);
        for(int i = 0;i < joint_num;i++)
        {
            body.leg.joint_data[i].kp = 0.0;
            body.leg.joint_data[i].kd = 0.0;
        }

        // for(int i = 0;i < joint_num;i++)
        // {
        //     body.leg.endp_tar[i] = - k_spring * (1 - l_0/body.leg.len) * body.leg.endp[i];
        //     if(body.leg.endv[2] > 0)
        //         body.leg.endp_tar[i]*= 12.5;
        // }
        // // body.leg.endf_tar[0]*= 1.35;
        // body.leg.endp_tar[2]+= l_0;
        // delta.InverseKinematics(body.leg);
        // for(int i = 0;i < joint_num;i++)
        // {
        //     body.leg.joint_data[i].kp = 5.0;
        //     body.leg.joint_data[i].kd = 0.1;
        // }
    }
    
    void JumpController::Init(double l_0_,double k_spring_,double margin_)
    {
        l_0 = l_0_;
        k_spring = k_spring_;
        margin = margin_;
        first_minimum = 0;
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
    joint_data[(*motor_msg).id - 1].pos = (*motor_msg).pos;
    joint_data[(*motor_msg).id - 1].vel = (*motor_msg).vel;
    joint_data[(*motor_msg).id - 1].tor = (*motor_msg).tor;
    joint_data[(*motor_msg).id - 1].tem = (*motor_msg).tem;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg,double* orient,double* acc)
{
    orient[0] = (*imu_msg).orientation.x;
    orient[1] = (*imu_msg).orientation.y;
    orient[2] = (*imu_msg).orientation.z;
    orient[3] = (*imu_msg).orientation.w;
    acc[0] = (*imu_msg).linear_acceleration.x;
    acc[1] = (*imu_msg).linear_acceleration.y;
    acc[2] = (*imu_msg).linear_acceleration.z;
}

int main(int argc, char **argv)
{
    /* variables */
    int countl = 0, loop_hz = 500;
    const uint16_t jointnum = 3;
    controller::motor_data joint[jointnum];
    double orientation[4],acceleration[3];

    /* ros */
    ros::init(argc, argv, "JumpController");

    ros::NodeHandle nh;

    ros::Publisher motor_pub = 
    nh.advertise<can::motor_data>("cybergear_cmds", jointnum);

    ros::Subscriber motor_sub = 
    nh.subscribe<can::motor_data>("cybergear_msgs", jointnum, boost::bind(&motorCallback,_1,joint));

    ros::Subscriber body_imu = 
    nh.subscribe<sensor_msgs::Imu>("imu/data", 1, boost::bind(&imuCallback,_1,orientation,acceleration));

    ros::Rate loop_rate(loop_hz);

    /* Initial */
    InitJoint(joint,jointnum);
    controller::JumpController jc(orientation,acceleration,62.5,40.0,110.0,250.0,joint,jointnum,2.5,0.1);
    jc.Init(200,0.1,10);
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