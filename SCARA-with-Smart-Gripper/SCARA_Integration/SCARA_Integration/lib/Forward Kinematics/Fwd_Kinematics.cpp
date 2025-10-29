#include "Fwd_Kinematics.h"
#include <math.h>

Fwd_Kinematics::Fwd_Kinematics()
{
}

Fwd_Kinematics::~Fwd_Kinematics()
{
}

void Fwd_Kinematics::setup(float DH_parameters[][4], uint8_t num_dof)
{
    _DH = DH_parameters;
    _num_dof = num_dof;
}

void Fwd_Kinematics::updateDH(uint8_t dof, DH parameter, float value)
{
    _DH[dof][parameter] = value;
}

void Fwd_Kinematics::calculate(float pose[6])
{
    // Inicializar una matriz acumulativa donde aparecer el HM_Total
    float HM_Total[4][4] = {{1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1}};
    // Aplicar la operacion por cada grado de libertad
    for (uint8_t i = 0; i < _num_dof; i++)
    {
        float HM_dof[4][4];
        HM_dof[0][0] = cos(_DH[i][THETA]*DEG2RAD);
        HM_dof[0][1] = -cos(_DH[i][ALPHA]*DEG2RAD)*sin(_DH[i][THETA]*DEG2RAD);
        HM_dof[0][2] = sin(_DH[i][ALPHA]*DEG2RAD)*sin(_DH[i][THETA]*DEG2RAD);
        HM_dof[0][3] = _DH[i][R]*cos(_DH[i][THETA]*DEG2RAD);

        HM_dof[1][0] = sin(_DH[i][THETA]*DEG2RAD);
        HM_dof[1][1] = cos(_DH[i][ALPHA]*DEG2RAD)*cos(_DH[i][THETA]*DEG2RAD);
        HM_dof[1][2] = -sin(_DH[i][ALPHA]*DEG2RAD)*cos(_DH[i][THETA]*DEG2RAD);
        HM_dof[1][3] = _DH[i][R]*sin(_DH[i][THETA]*DEG2RAD);

        HM_dof[2][0] = 0;
        HM_dof[2][1] = sin(_DH[i][ALPHA]*DEG2RAD);
        HM_dof[2][2] = cos(_DH[i][THETA]*DEG2RAD);
        HM_dof[2][3] = _DH[i][D];

        HM_dof[3][0] = 0;
        HM_dof[3][1] = 0;
        HM_dof[3][2] = 0;
        HM_dof[3][3] = 1;

        float HM_row_temp[4];
        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                HM_row_temp[column] = 0;
                for (int k = 0; k < 4; k++)
                    HM_row_temp[column] += HM_Total[row][k] * HM_dof[k][column];
            }
            // Actualizar cada renglon por su acumulativo ya que no se vuelven a utilzar en la multiplicacion
            for (int column = 0; column < 4; column++)
                HM_Total[row][column] = HM_row_temp[column];
        }

        float R11 = HM_Total[0][0];
        float R21 = HM_Total[1][0];
        float R31 = HM_Total[2][0];
        float R32 = HM_Total[2][1];
        float R33 = HM_Total[2][2];
        float R14 = HM_Total[0][3];
        float R24 = HM_Total[1][3];
        float R34 = HM_Total[2][3];

        float Alpha_Pose = atan2(R21,R11);
        float Beta_Pose = atan2(-R31,sqrt((pow(R11,2))+(pow(R21,2))));
        float Gamma_Pose = atan2(R32,R33);

        pose[0] = Alpha_Pose;
        pose[1] = Beta_Pose;
        pose[2] = Gamma_Pose;
        pose[3] = HM_Total[0][3];
        pose[4] = HM_Total[1][3];
        pose[5] = HM_Total[2][3];
    }
}