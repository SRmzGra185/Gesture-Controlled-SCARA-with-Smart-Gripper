#ifndef __INV_KINEMATICS_H__
#define __INV_KINEMATICS_H__

#include <math.h>

#define DEG2RAD 0.0174533F
#define RAD2DEG 57.29578F


class Inv_kinematics{
    private:
    public:
        Inv_kinematics();
        ~Inv_kinematics();
        void calculate(float Eslabones[2], float Posiciones[2], float ResultadosInversa[2]);
};
#endif // __INV_KINEMATICS_H__