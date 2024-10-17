#include "JumpController.h"
#include "can/motor_data.h"
#include "yaml-cpp/yaml.h"

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
        /* forward kinemtatics, calculate endp, endv, endf and Jacob */
        delta.CalJacob(body.leg[0]);
        delta.ForwardKinematicsVF(body.leg[0]);
        /* IMU for Body */
        QuattoRotMat(body.orient,body.rot_mat);
        QuattoEuler(body.orient,body.roll,body.pitch,body.yaw);
        delta.InverseMatrix33(body.rot_mat,body.rerot_mat);
        /* acc, dv and pos */
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
        /* highest height during last hop */
        if(state_height < body.pos[2])
            state_height = body.pos[2];
        /* end position in world coordinate*/
        for(int i = 0;i < 3;i++)
        {
            body.leg[0].endp_wd[i] = 0.0;
            body.leg[0].endp_wd[i]-= body.rot_mat[i][0] * body.leg[0].endp[0] / 1000.0;
            body.leg[0].endp_wd[i]+= body.rot_mat[i][1] * body.leg[0].endp[1] / 1000.0;
            body.leg[0].endp_wd[i]-= body.rot_mat[i][2] * body.leg[0].endp[2] / 1000.0;
            body.leg[0].endp_wd[i]+= body.pos[i];
        }
        body.leg[0].endp_wd[2]-= body.rot_mat[2][2] * 0.0345;
    }

    void JumpController::JudgeState(controller::body_data& body,controller::user_data& user)
    {
        if(body.state == stateFlying)
        {
            if(body.leg[0].len < l_0 * 0.96)
            {
                last_len = body.leg[0].len;
                height_err = user.pos[2] - state_height;
                integer_height_err+= height_err;
                ROS_INFO("  hop height:%lf",state_height);
                // ROS_INFO(" integer_err:%lf",integer_height_err);
                state_height = 0;
                ROS_INFO("    body vel:%lf",body.vel[0]);
                ROS_INFO("stateLanding");
                body.state = stateLanding;
            }
        }
        else
        {
            if(body.leg[0].len >= l_0)
            {
                if(!first_jump)
                    first_jump = 1;
                ROS_INFO(" stance time:%lf",stance_time);
                last_stance_time = stance_time;
                stance_time = 0;
                ROS_INFO("  max torque:%lf",max_torque);
                max_torque = 0;
                ROS_INFO(" stateFlying");
                body.state = stateFlying;
                
                body.pos[2] = 0.0;
                body.pos[2]+= body.rot_mat[2][0] * body.leg[0].endp[0] / 1000.0;
                body.pos[2]-= body.rot_mat[2][1] * body.leg[0].endp[1] / 1000.0;
                body.pos[2]+= body.rot_mat[2][2] * body.leg[0].endp[2] / 1000.0;
            }
        }
    }

    void JumpController::SetFlyingAngle(controller::body_data& body,controller::user_data& user)
    {
        // forward velocity - flight position
        pe_wd[0] = 1000.0 * (body.vel[0] * last_stance_time / 2.0 - base_vel_kp * (user.vel[0] - body.vel[0]));
        pe_wd[1] = 1000.0 * (body.vel[1] * last_stance_time / 2.0 - base_vel_kp * (user.vel[1] - body.vel[1]));
        pe_wd[2] = -sqrt(l_0 * l_0 - pe_wd[0] * pe_wd[0] - pe_wd[1] * pe_wd[1]);
        // end position
        for(int i = 0;i < 3;i++)
        {
            body.leg[0].endp_tar[i] = 0.0;
            body.leg[0].endp_tar[i]-= body.rerot_mat[i][0] * pe_wd[0];
            body.leg[0].endp_tar[i]+= body.rerot_mat[i][1] * pe_wd[1];
            body.leg[0].endp_tar[i]-= body.rerot_mat[i][2] * pe_wd[2];
        }
    
        delta.InverseKinematics(body.leg[0]);
        for(int i = 0;i < body.leg[0].joint_num;i++)
        {
            // body.leg[0].joint_data[i].pos_tar = 0.0;
            body.leg[0].joint_data[i].vel_tar = 0.0;
            body.leg[0].joint_data[i].tor_tar = 0.0;
            body.leg[0].joint_data[i].kp = joint_kp;
            body.leg[0].joint_data[i].kd = joint_kd;
        }
    }

    void JumpController::SetLandingForce(controller::body_data& body,controller::user_data& user)
    {
        stance_time+= 1.0 / ctrl_rate;
        // set axial force like spring
        for(int i = 0;i < body.leg[0].joint_num;i++)
        {
            body.leg[0].endf_tar[i] = -k_spring * (1 - l_0/body.leg[0].len) * body.leg[0].endp[i];
        }
        // base attitude control
        // fe_wd[0] = last_stance_time * (-base_att_kp * body.pitch - base_att_kd * body.ang_vel[1]);
        // fe_wd[1] = last_stance_time * (base_att_kp * body.roll + base_att_kd * body.ang_vel[0]);
        fe_wd[0] = -base_att_kp * body.pitch - base_att_kd * body.ang_vel[1];
        fe_wd[1] = base_att_kp * body.roll + base_att_kd * body.ang_vel[0];
        // height PI control
        if(body.leg[0].len > last_len && first_jump)
        {
            fe_wd[2] = - (base_hei_kp * height_err + base_hei_kd * integer_height_err);
            if(fabs(fe_wd[0]) > friction * fabs(fe_wd[2]))
            {
                if(fe_wd[0] > 0.0)
                    fe_wd[0] = friction * fabs(fe_wd[2]);
                else
                    fe_wd[0] = -friction * fabs(fe_wd[2]);
            }
        }
        last_len = body.leg[0].len;
        for(int i = 0;i < 3;i++)
        {
            body.leg[0].endf_tar[i]-= body.rerot_mat[i][0] * fe_wd[0];
            body.leg[0].endf_tar[i]+= body.rerot_mat[i][1] * fe_wd[1];
            body.leg[0].endf_tar[i]-= body.rerot_mat[i][2] * fe_wd[2];
        }

        delta.Statics(body.leg[0]);
        for(int i = 0;i < body.leg[0].joint_num;i++)
        {
            if(fabs(max_torque) < fabs(body.leg[0].joint_data[i].tor_tar))
                max_torque = body.leg[0].joint_data[i].tor_tar;
            body.leg[0].joint_data[i].kp = 0.0;
            body.leg[0].joint_data[i].kd = 0.001;
        }
    }

    void JumpController::Controller(controller::body_data& body,controller::user_data& user)
    {
        UpdateParam(body);
        JudgeState(body,user);
        if(body.state == stateFlying)
            SetFlyingAngle(body,user);
        else
            SetLandingForce(body,user);
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
    body_data.state = controller::stateFlying;
    for(int i = 0;i < 3;i++)
    {
        body_data.raw_acc[i] = 0.0;
        body_data.acc[i] = 0.0;
        body_data.vel[i] = 0.0;
        body_data.pos[i] = 0.0;
    }
    /* set initial z */
    body_data.pos[2] = 0.3465;
}

void InitUser(controller::user_data& user_date){
    user_date.pos[2] = 0.300;
    user_date.vel[0] = 0.0;
    user_date.vel[1] = 0.0;
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
    int countl = 0, countsum = 0;
    int loop_hz = 500, pause = 1;
    const int legnum = 1;
    const int jointnum = 3;
    controller::body_data body;
    controller::leg_data leg[legnum];
    controller::motor_data joint[jointnum];
    controller::user_data user;

    std::string param_file = "./src/Quata/controller/src/param.yaml";
    YAML::Node yamlConfig = YAML::LoadFile(param_file);

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

    ros::Publisher basepos =
    nh.advertise<geometry_msgs::Point>("basepos", 1);

    /* Init */
    // parameters refer to mechanism
    InitBody(joint, jointnum, leg, legnum, body, 62.5, 40.0, 110.0, 250.0);
    InitUser(user);
    // parameters refer to control
    controller::JumpController jc(
    yamlConfig["l_0"].as<double>(),
    yamlConfig["k_spring"].as<double>(),
    yamlConfig["margin"].as<double>(),
    yamlConfig["friction"].as<double>(),
    yamlConfig["base_vel_kp"].as<double>(),
    yamlConfig["base_hei_kp"].as<double>(),
    yamlConfig["base_hei_kd"].as<double>(),
    yamlConfig["base_att_kp"].as<double>(),
    yamlConfig["base_att_kd"].as<double>(),
    yamlConfig["joint_kp"].as<double>(),
    yamlConfig["joint_kd"].as<double>(),
    loop_hz
    );
    can::motor_data motor_cmd;
    geometry_msgs::Point bpos;

    /* loop */
    while (ros::ok())
    {
        ros::spinOnce();   
        if(pause)
            continue;
        else
        {
            jc.Controller(body,user);
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

            bpos.x = body.leg[0].endp_wd[0];
            bpos.y = body.leg[0].endp_wd[1];
            bpos.z = body.leg[0].endp_wd[2];
            basepos.publish(bpos);

            countl++;
            countsum++;
            if(countl == loop_hz)
            {
                countl = 0;
                // ROS_INFO("One Loop");
            }
            // if(countsum > 3 * loop_hz)
            // {
            //     user.vel[0] = 0.1;
            // }
        }
        loop_rate.sleep();
    }
    return 0;
}