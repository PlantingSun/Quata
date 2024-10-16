#ifndef DELTAMECHANISM_H
#define DELTAMECHANISM_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "Data.h"

namespace controller{
    /* Delta */
class Delta{
        public:
            void InverseMatrix33(double (*mat_in)[3],double (*mat_out)[3]);
            void InverseKinematics(struct leg_data& leg);
            void ForwardKinematicsP(struct leg_data& leg);
            void ForwardKinematicsVF(struct leg_data& leg);
            void CalJacob(struct leg_data& leg);
            void CalreJacob(struct leg_data& leg);
            void Statics(struct leg_data& leg);
        private:
            void VectorSub3(double* arr_front,double* arr_back,double* arr_out);
            void MatrixSub33(double (*mat_front)[3],double (*mat_back)[3],double (*mat_out)[3]);
            void CrossProduct3(double* arr_front,double* arr_back,double* arr_out);
            double Norm3(double* arr);
            void Norm3d(double* arr,double (*arr_d)[3],double len,double* len_d);
            const double sqr3 = 1.73205081;
            const double pi2_3 = M_PI * 2.0 / 3.0;
    };
}

#endif
