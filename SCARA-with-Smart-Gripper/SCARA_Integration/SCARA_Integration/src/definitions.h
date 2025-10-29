#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <Arduino.h>
#include <math.h>
#include <InterconxionMatlabVSC.h>
#include <Fwd_Kinematics.h>
#include <BluetoothSerial.h>
#include <PID.h>
#include <Encoders.h>
#include <HBridge.h>

float Kp = 2.00;
float Ki = 0.00;
float Kd = 0.00;

#define MotorBomba1 16
#define MotorBomba2 4



uint8_t Entrada_Motor[] = {17, 13};
uint8_t Canales_Motor[] = {1, 0};
//uint8_t Entradas_Bomba[] = {16,4};
uint8_t Canales_Bomba[] = {0,1};
int pines_cuadratura[] = {34, 33};
int num_canales = 2;
uint16_t timeout_ms = 5;
ulong current_time, prev_time;
float PPR = 341.2;
float DPA = 360/PPR; //Degrees per angle.
// Definir las variables del PID
float ref;


PID pid[1];  //Cantidad de PID que se necesitan //En teroria deberia de ser 4
unsigned int dt_ms = 10;

//Instancias de todas las clases (librerias).
HBridge dc;
PID control;
Encoders cuadratura;

#endif // __DEFINITIONS_H__