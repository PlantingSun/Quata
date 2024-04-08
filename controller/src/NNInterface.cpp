#include "NNInterface.h"

namespace controller{
    /* public */
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
