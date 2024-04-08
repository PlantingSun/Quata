#include "JumpController.h"
#include "can/motor_data.h"

/* for namespace */
namespace controller
{   
    void JumpController::QuattoRotMat(double* quat,double (*rot_mat)[3])
    {
        double xsq = quat[1] * quat[1];
        double ysq = quat[2] * quat[2];
        double zsq = quat[3] * quat[3];
        double xyt = quat[1] * quat[2];
        double xzt = quat[1] * quat[3];
        double xwt = quat[1] * quat[0];
        double yzt = quat[2] * quat[3];
        double ywt = quat[2] * quat[0];
        double zwt = quat[3] * quat[0];

        rot_mat[0][0] = 1.0 - 2.0 * ysq - 2.0 * zsq;
        rot_mat[0][1] = 2.0 * xyt - 2.0 * zwt;
        rot_mat[0][2] = 2.0 * xzt + 2.0 * ywt;
        rot_mat[1][0] = 2.0 * xyt + 2.0 * zwt;
        rot_mat[1][1] = 1.0 - 2.0 * xsq - 2.0 * zsq;
        rot_mat[1][2] = 2.0 * yzt - 2.0 * xwt;
        rot_mat[2][0] = 2.0 * xzt - 2.0 * ywt;
        rot_mat[2][1] = 2.0 * yzt + 2.0 * xwt;
        rot_mat[2][2] = 1.0 - 2.0 * xsq - 2.0 * ysq;
    }

    void JumpController::QuattoEuler(double* quat,double& roll,double& pitch,double& yaw)
    {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (quat[0] * quat[1] + quat[2] * quat[3]);
        double cosr_cosp = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        // pitch (y-axis rotation)
        double sinp = 2.0 * (quat[0] * quat[2] - quat[3] * quat[1]);
        pitch = std::asin(sinp);
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (quat[0] * quat[3] + quat[1] * quat[2]);
        double cosy_cosp = 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void JumpController::UpdateParam(controller::body_data& body)
    {
        delta.CalJacob(body.leg[0]);
        delta.ForwardKinematicsVF(body.leg[0]);

        /* IMU for Body */
        QuattoRotMat(body.orient,body.rot_mat);
        QuattoEuler(body.orient,body.roll,body.pitch,body.yaw);

        // printf("yaw:%lf pitch:%lf roll:%lf\n",body.yaw,body.pitch,body.roll);

        for(int i = 0;i < 3;i++)
        {
            body.acc[i] = 0.0;
            body.vel[i] = 0.0;
            for(int j = 0;j < 3;j++)
            {
                body.acc[i]+= body.rot_mat[i][j] * body.raw_acc[j];
                body.vel[i]+= body.rot_mat[i][j] * body.raw_vel[j];
                body.ang_vel[i]+= body.rot_mat[i][j] * body.raw_ang_vel[j];
            }
            body.pos[i]+= body.vel[i] / ctrl_rate;
        }
    }

    void JumpController::JudgeState(controller::body_data& body)
    {
        // if(first_minimum)
        // {
            double pos2 = 0.0;
            for(int i = 0;i < 3;i++)
                pos2+= body.rot_mat[2][i] * body.leg[0].endp[i] / 1000.0;
            
            if(body.pos[2] - pos2 > margin)
                body.state = stateFlying;
            else
                body.state = stateLanding;

            printf("%lf\n",body.pos[2] - pos2);
        // }
        // else
        // {
        //     body.state = stateLanding;
        //     if(body.vel[2] > 0)
        //     {
        //         first_minimum = 1;
        //         // body.pos[2] = body.leg[0].endp[2];
        //         body.pos[2] = 0.0;
        //         for(int i = 0;i < 3;i++)
        //             body.pos[2]+= body.rot_mat[2][i] * body.leg[0].endp[i];
        //     }
        // }
    }

    void JumpController::SetFlyingAngle(controller::body_data& body)
    {
        for(int i = 0;i < 3;i++)
        {
            body.leg[0].joint_data[i].pos_tar = 0.0;
            body.leg[0].joint_data[i].vel_tar = 0.0;
            body.leg[0].joint_data[i].tor_tar = 0.0;
            body.leg[0].joint_data[i].kp = 2.5;
            body.leg[0].joint_data[i].kd = 0.02;
        }
    }

    void JumpController::SetLandingForce(controller::body_data& body)
    {
        // set axial force
        for(int i = 0;i < body.leg[0].joint_num;i++)
        {
            body.leg[0].endf_tar[i] = - k_spring * (1 - l_0/body.leg[0].len) * body.leg[0].endp[i];
        }
        body.leg[0].endf_tar[2]+= 12.0;

        // set radial force
        double rad_t_x = - k_spring * body.pitch;
        double rad_t_y = - k_spring * body.roll;
        double rad_f_x = - rad_t_x * 5000.0 / body.leg[0].len;
        double rad_f_y = - rad_t_y * 5000.0 / body.leg[0].len;

        body.leg[0].endf_tar[2]-= rad_f_x * body.leg[0].endp[0] / body.leg[0].len;
        body.leg[0].endf_tar[0]+= rad_f_x * body.leg[0].endp[2] / body.leg[0].len;

        body.leg[0].endf_tar[2]-= rad_f_y * body.leg[0].endp[1] / body.leg[0].len;
        body.leg[0].endf_tar[1]+= rad_f_y * body.leg[0].endp[2] / body.leg[0].len;

        delta.Statics(body.leg[0]);

        for(int i = 0;i < body.leg[0].joint_num;i++)
        {
            body.leg[0].joint_data[i].kp = 0.0;
            body.leg[0].joint_data[i].kd = 0.02;
        }
    }

    void JumpController::ApplyRLNetwork(controller::body_data& body)
    {
        input[0] = body.vel[0]; input[1] = body.vel[1]; input[2] = body.vel[2];
        input[3] = body.ang_vel[0]; input[4] = body.ang_vel[1]; input[5] = body.ang_vel[2];
        input[9] = 0.0; input[10] = 0.0; input[11] = 0.0;
        input[12] = body.leg[0].endp[0];
        input[13] = body.leg[0].endp[1];
        input[14] = body.leg[0].endp[2] - l_0;
        input[15] = body.leg[0].endv[0];
        input[16] = body.leg[0].endv[1];
        input[17] = body.leg[0].endv[2];
        input[18] = output[0];
        input[19] = output[1];
        input[20] = output[2];
        nninterface->NNForward(input,output);
    }

    void JumpController::Controller(controller::body_data& body)
    {
        UpdateParam(body);
        // JudgeState(body);
        // if(body.state == stateFlying)
        // {
        //     SetFlyingAngle(body);
        //     // SetLandingForce(body);
        // }
        // else
        // {
        //     SetLandingForce(body);
        // }

        // SetLandingForce(body);
    }
}


/* for node */
void InitBody(controller::motor_data* joint_data,int joint_num,
              controller::leg_data* leg,int leg_num,
              controller::body_data& body_data,
              double rbody_,double rfoot_,double uppleglen_,double lowleglen_)
{
    for(int i = 1;i <= joint_num;i++)
    {
        joint_data[i - 1].ID = i;
        joint_data[i - 1].pos_tar = 0.0;
        joint_data[i - 1].vel_tar = 0.0;
        joint_data[i - 1].tor_tar = 0.0;
    }

    for(int i = 1;i <= leg_num;i++)
    {
        leg[i - 1].joint_data = joint_data;
        leg[i - 1].joint_num = joint_num;
        leg[i - 1].rbody = rbody_;
        leg[i - 1].rfoot = rfoot_;
        leg[i - 1].rdif = rbody_ - rfoot_;
        leg[i - 1].uppleglen = uppleglen_;
        leg[i - 1].lowleglen = lowleglen_;
    }

    body_data.leg = leg;
    body_data.leg_num = leg_num;
    for(int i = 0;i < 3;i++)
    {
        body_data.raw_acc[i] = 0.0;
        body_data.acc[i] = 0.0;
        body_data.vel[i] = 0.0;
        body_data.pos[i] = 0.0;
    }
    /* set initial z */
    body_data.pos[2] = 0.50;
}

void motorCallback(const can::motor_data::ConstPtr& motor_msg,controller::motor_data* joint_data)
{
    joint_data[(*motor_msg).id - 1].pos = (*motor_msg).pos;
    joint_data[(*motor_msg).id - 1].vel = (*motor_msg).vel;
    joint_data[(*motor_msg).id - 1].tor = (*motor_msg).tor;
    joint_data[(*motor_msg).id - 1].tem = (*motor_msg).tem;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg,controller::body_data* body_data)
{
    (*body_data).orient[0] = (*imu_msg).orientation.w;
    (*body_data).orient[1] = (*imu_msg).orientation.x;
    (*body_data).orient[2] = (*imu_msg).orientation.y;
    (*body_data).orient[3] = (*imu_msg).orientation.z;
    (*body_data).raw_ang_vel[0] = (*imu_msg).angular_velocity.x;
    (*body_data).raw_ang_vel[1] = (*imu_msg).angular_velocity.y;
    (*body_data).raw_ang_vel[2] = (*imu_msg).angular_velocity.z;
    (*body_data).raw_acc[0] = (*imu_msg).linear_acceleration.x;
    (*body_data).raw_acc[1] = (*imu_msg).linear_acceleration.y;
    (*body_data).raw_acc[2] = (*imu_msg).linear_acceleration.z;
}

void dvCallback(const geometry_msgs::Vector3Stamped::ConstPtr& dv_msg,controller::body_data* body_data)
{
    (*body_data).raw_vel[0] = (*dv_msg).vector.x;
    (*body_data).raw_vel[1] = (*dv_msg).vector.y;
    (*body_data).raw_vel[2] = (*dv_msg).vector.z;
}

void pauseCallback(const std_msgs::String::ConstPtr& pause_msg,int* pause_flag)
{
    if(*(*pause_msg).data.c_str() == '1')
        (*pause_flag) = 1;
    else
        (*pause_flag) = 0;
}

int main(int argc, char **argv)
{
    /* variables */
    int countl = 0, loop_hz = 500, pause = 1;
    const int legnum = 1;
    const int jointnum = 3;
    controller::body_data body;
    controller::leg_data leg[legnum];
    controller::motor_data joint[jointnum];

    /* ros */
    ros::init(argc, argv, "JumpController");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = 
    nh.advertise<can::motor_data>("cybergear_cmds", jointnum);
    ros::Subscriber motor_sub = 
    nh.subscribe<can::motor_data>("cybergear_msgs", jointnum, boost::bind(&motorCallback,_1,joint));
    ros::Subscriber body_imu = 
    nh.subscribe<sensor_msgs::Imu>("imu/data", 1, boost::bind(&imuCallback,_1,&body));
    ros::Subscriber body_dv = 
    nh.subscribe<geometry_msgs::Vector3Stamped>("imu/dv", 1, boost::bind(&dvCallback,_1,&body));
    ros::Subscriber pause_sub = 
    nh.subscribe<std_msgs::String>("pause", 1, boost::bind(&pauseCallback,_1,&pause));
    ros::Rate loop_rate(loop_hz);

    /* Init */
    InitBody(joint, jointnum, leg, legnum, body, 62.5, 40.0, 110.0, 250.0);
    //TODO
    controller::NNInterface nn(1.0,2.0,1.0,1.0,1.0);
    controller::JumpController jc(212.0, 1.0, 0.05, 1.0, 0.012, loop_hz, &nn);
    can::motor_data motor_cmd;

    /* loop */
    while (ros::ok())
    {
        ros::spinOnce();
        if(pause)
            continue;
        else
        {
            jc.Controller(body);
            for(int i = 0;i < jointnum;i++)
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
                // ROS_INFO("One Loop");
            }
        }
        loop_rate.sleep();
    }
    return 0;
}