#ifndef NNINTERFACE_H
#define NNINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <iostream>
// #include <torch/script.h>
#include <Eigen/Dense>

namespace controller{
    class NNInterface{
        public:
            NNInterface(double lin_vel_scale_,double ang_vel_scale_,double commands_scale_,
                        double pos_scale_,double vel_scale_)
            {
                lin_vel_scale = lin_vel_scale_;
                ang_vel_scale = ang_vel_scale_;
                commands_scale = commands_scale_;
                pos_scale = pos_scale_;
                vel_scale = vel_scale_;
            }
            void Init();
            void NNForward(double* input,double* output);
        private:
            void BuildInput();
            void SetScale();
            void BuildOutput();
            double lin_vel_scale,ang_vel_scale,commands_scale;
            double pos_scale,vel_scale;

            // torch::jit::script::Module module;
            // std::vector<torch::jit::IValue> inputs;
            // std::vector<torch::jit::IValue> outputs;
    };
}

#endif
