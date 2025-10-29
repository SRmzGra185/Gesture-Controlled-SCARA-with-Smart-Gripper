#include "Inv_kinematics.h"
#include <math.h>

// INVERSE KINEMATICS
Inv_kinematics::Inv_kinematics()
{
  
}

Inv_kinematics::~Inv_kinematics()
{
  
}

void Inv_kinematics::calculate(float Eslabones[2], float Posicion[2], float ResultadosInversa[2])
{

  //_Posicion[0] = _Eslabones[0]*cos(_Theta1*DEG2RAD) + _Eslabones[1]*cos(_Theta1*DEG2RAD + _Theta2*DEG2RAD);
  //_Posicion[1] = _Eslabones[0]*cos(_Theta1*DEG2RAD) + _Eslabones[1]*cos(_Theta1*DEG2RAD + _Theta2*DEG2RAD);
  float P = sqrt(pow(Posicion[0],2)+pow(Posicion[1],2));
  float Beta = RAD2DEG*acos(((pow(P,2))-(pow(Eslabones[0],2))-(pow(Eslabones[1],2)))/(-2*(Eslabones[0])*(Eslabones[1])));
  float Alpha = RAD2DEG*acos(((pow(Eslabones[1],2))-(pow(Eslabones[0],2))-(pow(P,2)))/(-2*(Eslabones[0])*P));
  float Gamma = RAD2DEG*atan(Posicion[1]/Posicion[0]);

  float Theta2 = 180 - Beta;
  float Theta1 = Gamma - Alpha;

  ResultadosInversa[0] = Theta1;
  ResultadosInversa[1] = Theta2;
}



