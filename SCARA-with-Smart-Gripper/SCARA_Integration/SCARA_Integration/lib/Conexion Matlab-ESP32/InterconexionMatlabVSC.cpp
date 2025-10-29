#include <InterconxionMatlabVSC.h>
#include <SoftwareSerial.h>

InterconexionMatlabVSC::InterconexionMatlabVSC()
{
}

InterconexionMatlabVSC::~InterconexionMatlabVSC()
{
}

void InterconexionMatlabVSC::EleccionCaso(char TipoSerial, float AnguloJoint1, float AnguloJoint2, float PX, float PY, float PZ, float Proportional, float Integral, float Derivative, int Flag)
{

    
    switch (TipoSerial)
    {
    case 'A':                         // Forward Kinematics Joint 1
    {
        AnguloJoint1 = Serial.readStringUntil('\n').toFloat();    // Lee AnguloJoint1
        //AnguloJoint1 = CadenaA.toFloat();
        Serial.write(int(AnguloJoint1));   // Envia A
        //return AnguloJoint1;
        break;
    }
    case 'B': 
    {                           // Forward Kinematics Joint 2
        AnguloJoint2 = Serial.readStringUntil('\n').toFloat();   // Lee AnguloJoint1
        //AnguloJoint2 = CadenaB.toFloat();
        Serial.write(int(AnguloJoint2)); // Envia A
        //return AnguloJoint2;
        break;
    }
    case 'X':               // Inverse Kinematics X
    {
        PX = Serial.readStringUntil('\n').toFloat();    // Lee AnguloJoint1
        //PX = CadenaX.toFloat();
        //PX = Serial.parseFloat();
        Serial.write(int(PX));   // Envia PX
        //return PX;
        break;
    }
    case 'Y':               // Inverse Kinematics Y
    {
        PY = Serial.readStringUntil('\n').toFloat();    // Lee AnguloJoint1
        // PY = CadenaY.toFloat();
        //PY = Serial.parseFloat();
        Serial.write(int(PY));   // Envia PY
        //return PY;
        break;
    }

    case 'Z': // Inverse Kinematics Z
    {
        PZ = Serial.readStringUntil('\n').toFloat();    // Lee AnguloJoint1
        //PZ = Serial.parseFloat();
        Serial.write(int(PZ));   // Envia PZ
        //return PZ;
        break;
    }
    case 'P':                         // PID Controller - Proportional
    {
        String CadenaP = Serial.readStringUntil('\n');    // Lee AnguloJoint1
        //Proportional = Serial.read();
        Proportional = CadenaP.toFloat();
        Serial.write(int(Proportional));   // Envia P
        //return Proportional;
        break;
    }
    case 'I':                     // PID Controller - Integral
    {
        String CadenaI = Serial.readStringUntil('\n');   // Lee AnguloJoint1
        //Integral = Serial.read();
        Integral = CadenaI.toFloat();
        Serial.write(int(Integral));   // Envia I
        //return Integral;
        break;
    }
    case 'D':                       // PID Controller - Derivative
    {
        String CadenaD = Serial.readStringUntil('\n');    // Lee AnguloJoint1
        //Derivative = Serial.read();
        Derivative = CadenaD.toFloat();
        Serial.write(int(Derivative));   // Envia D
        //return Derivative;
        break;
    }
    default:
        break;
    }
}