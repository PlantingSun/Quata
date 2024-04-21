#include "NNInterface.h"

Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    Eigen::Vector3d q_vec = q.head<3>();
    double q_w = q(3);

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v)) * 2.0;

    return a - b + c;
}

namespace controller{
    /* public */
    void NNInterface::Init()
    {
        // module = 
        // torch::jit::load("/home/tsingchen/quata_ws/src/Quata/resources/RLSLIP/model_900.pt");
        // torch::Tensor tensor = torch::rand({2, 3});
        // std::cout << tensor << std::endl;

        // torch::jit::script::Module slip = torch::jit::load("/home/tsingchen/quata_ws/src/Quata/resources/RLSLIP/model_900.pt");
        
        std::cout<<"load ok";
    }
    
    void NNInterface::NNForward(double* input,double* output)
    {
        BuildInput();
        SetScale();

        BuildOutput();
    }
    /* private */
    void NNInterface::BuildInput()
    {

    }
    void NNInterface::SetScale()
    {

    }
    void NNInterface::BuildOutput()
    {

    }
}
