#include <definitions.h>

// Declaracion de Variables de Motores
#define PrincipalStep_PIN 23
#define PrincipalDir_PIN 19
#define BrazoStep_PIN 18
#define BrazoDir_PIN 5
#define VerticalStep_PIN 2
#define VerticalDir_PIN 15

int PasosPorRevolucion = 200;
int Direccion = 1;
int Grados = 5760; // Giro 360°

// Declaracion de Variables para IU
char TipoSerial;
float PX[1];
float PY[1];
float PZ[1];
float Proportional[1];
float Integral[1];
float Derivative[1];
int AccionEnProceso = 0;
int Flag = 1;
int FlagIK;
int FlagFK;
// Declaracion de Variables para Cinematica Directa;
float Pose[6];
int DoF = 3;
float AnguloJoint1[1];
float AnguloJoint2[1];
// Declaracion de Variables de Cinematica Inversa
float Eslabones[2] = {22.8, 13.6};
float Posicion[2];
float AngulosSalida[2];
// Declaracion de Variables de Rompecabezas
float CoordenadasPieza[2] = {10, 10};
float CoordenadasRack_Circulo[2] = {20, 20};
int TipoPieza;
float TomaPieza[1];
// Declaracion de Variables de PID
float Ganancias[3];

// Definicion de Librerias
InterconexionMatlabVSC VSC_MATLAB;
Fwd_Kinematics CinematicaDirecta;
Inv_kinematics CinematicaInversa;
MotoresStepper Stepper;
BluetoothSerial ConexionBluetooth;

void setup()
{
  // Se declara del Baud Rate de la ESP32
  Serial.begin(115200);
  ConexionBluetooth.begin("PedroPedro_MarkII");

  pinMode(PrincipalStep_PIN, OUTPUT);
  pinMode(PrincipalDir_PIN, OUTPUT);
  pinMode(BrazoDir_PIN, OUTPUT);
  pinMode(BrazoStep_PIN, OUTPUT);
  pinMode(VerticalDir_PIN, OUTPUT);
  pinMode(VerticalStep_PIN, OUTPUT);
  pinMode(MotorBomba1, OUTPUT);
  pinMode(MotorBomba2, OUTPUT);

  float gains[3] = {Kp, Ki, Kd};

  dc.setup(Entrada_Motor, Canales_Motor);
  cuadratura.setup(pines_cuadratura, DPA, num_canales, timeout_ms);
  control.setup(gains, dt_ms);
  prev_time = millis();
}
void loop()
{
  // Sección I - Recepción de Datos por Puerto Serial de MATLAB

  if (Serial.available())
  {
    char buffer[80];
    Serial.readStringUntil('\n').toCharArray(buffer, sizeof(buffer));
    sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &AnguloJoint1[0], &AnguloJoint2[0], &PX[0], &PY[0], &PZ[0], &Proportional[0], &Integral[0], &Derivative[0],&TomaPieza[0]);
    ConexionBluetooth.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n", AnguloJoint1[0], AnguloJoint2[0], PX[0], PY[0], PZ[0], Proportional[0], Integral[0], Derivative[0], TomaPieza[0]);
  }
  // Seccion II - Seccion de Cinematica Inversa con Valores de MATLAB
  if (PX[0] != 0 and PY[0] != 0)
  {
    Posicion[0] = PX[0];
    Posicion[1] = PY[0];
    CinematicaInversa.calculate(Eslabones, Posicion, AngulosSalida);
    delay(2500);
    Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), PasosPorRevolucion, PrincipalDir_PIN, 0, PrincipalStep_PIN);
    delay(2500);
    Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), PasosPorRevolucion, BrazoDir_PIN, Direccion, BrazoStep_PIN);
    ConexionBluetooth.printf("PX=%f ,PY=%f ,PZ=%f ,Theta1=%f, Theta2=%f\n", PX[0], PY[0], PZ[0], AngulosSalida[0], AngulosSalida[1]);
    PX[0] = 0;
    PY[0] = 0;
  }
  // Sección III - Rerepción de Datos de FK y calculo de variables de Posicion
  if (AnguloJoint1[0] > 0 and AnguloJoint2[0] > 0)
  {
    float ParametrosDH[][4] = {{AnguloJoint1[0], 30, 0, 0}, {AnguloJoint2[0], 20, 0, 0}};
    CinematicaDirecta.setup(ParametrosDH, DoF);
    CinematicaDirecta.calculate(Pose);
    Stepper.MotorGiroPrincipal(abs(16 * AnguloJoint1[0]), PasosPorRevolucion, PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
    Stepper.MotorBrazo(abs(16 * AnguloJoint2[0]), PasosPorRevolucion, BrazoDir_PIN, Direccion, BrazoStep_PIN);
    ConexionBluetooth.printf("Alpha=%f, Beta=%f, Gamma=%f, PX=%f ,PY=%f ,PZ=%f ,Theta1=%f, Theta2=%f\n", Pose[0], Pose[1], Pose[2], Pose[3], Pose[4], Pose[5], AnguloJoint1[0], AnguloJoint2[0]);
     AnguloJoint1[0] = 0;
    AnguloJoint2[0] = 0;
  }

  // Seccion IV - Recepcion de Datos de PID y cambio de PID para DC
  if (Proportional != 0 or Integral != 0 or Derivative != 0)
  {
    Ganancias[0] = Proportional[0];
    Ganancias[1] = Integral[0];
    Ganancias[2] = Derivative[0];
    control.setGains(Ganancias);
  }

  // Seccion V - Caso A - Algoritmo de Toma de Circulo
  if (TomaPieza[0] != 0)
  {
    Posicion[0] = 10;
    Posicion[1] = 10;
    CinematicaInversa.calculate(Eslabones, Posicion, AngulosSalida);
    delay(2500);
    Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), PasosPorRevolucion, PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
    delay(2500);
    Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), PasosPorRevolucion, BrazoDir_PIN, Direccion, BrazoStep_PIN);
    Stepper.MotorVertical(26000, PasosPorRevolucion, VerticalDir_PIN, 0, VerticalStep_PIN);
    digitalWrite(MotorBomba1, HIGH);
    digitalWrite(MotorBomba2, LOW);
    delay(5000);
    Stepper.MotorVertical(26000, PasosPorRevolucion, VerticalDir_PIN, 1, VerticalStep_PIN);
    digitalWrite(MotorBomba1, LOW);
    digitalWrite(MotorBomba2, LOW);
    CinematicaInversa.calculate(Eslabones, CoordenadasRack_Circulo, AngulosSalida);
    delay(2500);
    Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), PasosPorRevolucion, PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
    delay(2500);
    Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), PasosPorRevolucion, BrazoDir_PIN, Direccion, BrazoStep_PIN);
    delay(2500);
    Stepper.MotorVertical(4000, PasosPorRevolucion, VerticalDir_PIN, 0, VerticalStep_PIN);
    delay(2500);
    digitalWrite(MotorBomba1, LOW);
    digitalWrite(MotorBomba2, LOW);
    TomaPieza[0] = 0;
  }

  // Envio de Variables y Reinicio de Variables

  if (Serial.available() and AccionEnProceso == 1 and FlagIK == 1)
  {
    Serial.printf("PX=%f, PY=%f,PZ= %f,Theta1 = %f, Theta2=%f\n", PX[0], PY[0], PZ[0], AngulosSalida[0], AngulosSalida[1]);
    ConexionBluetooth.printf("%f,%f,%f,%f,%f\n", PX[0], PY[0], PZ[0], AngulosSalida[0], AngulosSalida[1]);
    AccionEnProceso = 0;
  }
  if (Serial.available() and AccionEnProceso == 1 and FlagFK == 1)
  {
    Serial.printf("PX= %f, PY=%f, PZ=%f,Theta1=%f,Theta2=%f\n", Pose[3], Pose[4], Pose[5], AnguloJoint1[0], AnguloJoint2[0]);
    ConexionBluetooth.printf("%f,%f,%f,%f,%f\n", Pose[3], Pose[4], Pose[5], AnguloJoint1[0], AnguloJoint2[0]);
    AccionEnProceso = 0;
  }
}