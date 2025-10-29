#ifndef __INTERCONXIONMATLABVSC_H__
#define __INTERCONXIONMATLABVSC_H__

#include <Arduino.h>
#include <Inv_kinematics.h>
#include <MotoresStepper.h>

class InterconexionMatlabVSC
{
public:                        // El usuario por fuera lo puede usar
    InterconexionMatlabVSC();  // Constructor
    ~InterconexionMatlabVSC(); // Deconstructor
    void EleccionCaso(char TipoSerial, float AnguloJoint1, float AnguloJoint2, float PX, float PY, float PZ, float Proportional, float Integral, float Derivative, int Flag);

private:   // Solo lo que esta dentro de la clase lo puede utilizar
protected: // Tiene acceso restingido
};

#endif // __INTERCONXIONMATLABVSC_H__