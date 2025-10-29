#ifndef __FWD_KINEMATICS_H__
#define __FWD_KINEMATICS_H__

#include <math.h>

#define DEG2RAD 0.0174533F
#define RAD2DEG 57.29578F

enum DH{
    THETA,
    D,
    ALPHA,
    R
};

class Fwd_Kinematics
{
    public:
        Fwd_Kinematics(); //Constructor de la biblioteca
        ~Fwd_Kinematics(); // Destructor de la Biblioteca
        void setup(float DH_parameters[][4], uint8_t num_dof);
        void updateDH(uint8_t dof, DH parameter, float value);
        void calculate (float pose[6]);

    private:
        uint8_t _num_dof;
        float (*_DH)[4];
};

#endif // __FWD_KINEMATICS_H__