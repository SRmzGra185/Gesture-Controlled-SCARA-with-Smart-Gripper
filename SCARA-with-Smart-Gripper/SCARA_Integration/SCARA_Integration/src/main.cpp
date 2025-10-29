#include <definitions.h>

// Motor Pin Definitions
#define PrincipalStep_PIN 23
#define PrincipalDir_PIN 19
#define BrazoStep_PIN 18
#define BrazoDir_PIN 5
#define VerticalStep_PIN 2
#define VerticalDir_PIN 15

int PasosPorRevolucion = 200;
int Direccion = 1;

// Control Mode
enum ControlMode {
    MODE_MATLAB = 0,
    MODE_KINECT = 1,
    MODE_BLUETOOTH = 2
};
ControlMode currentMode = MODE_KINECT;  // Default to Kinect mode

// Variables for control
float PX[1] = {0};
float PY[1] = {0};
float PZ[1] = {0};
float Proportional[1] = {0};
float Integral[1] = {0};
float Derivative[1] = {0};
float AnguloJoint1[1] = {0};
float AnguloJoint2[1] = {0};
float TomaPieza[1] = {0};

int AccionEnProceso = 0;
int Flag = 1;
int FlagIK = 0;
int FlagFK = 0;

// Cinematica Variables
float Pose[6];
int DoF = 3;
float Eslabones[2] = {22.8, 13.6};
float Posicion[2];
float AngulosSalida[2];
float CoordenadasPieza[2] = {10, 10};
float CoordenadasRack_Circulo[2] = {20, 20};
float Ganancias[3];

// Kinect-specific variables
unsigned long lastKinectUpdate = 0;
const unsigned long KINECT_TIMEOUT = 2000;  // 2 second timeout
bool kinectActive = false;
float smoothX = 0, smoothY = 0, smoothZ = 0;
const float SMOOTH_FACTOR = 0.3;  // Smoothing filter

// Library instances
InterconexionMatlabVSC VSC_MATLAB;
Fwd_Kinematics CinematicaDirecta;
Inv_kinematics CinematicaInversa;
MotoresStepper Stepper;
BluetoothSerial ConexionBluetooth;

void setup() {
    Serial.begin(115200);
    ConexionBluetooth.begin("SCARA_Kinect_Robot");
    
    // Motor pins
    pinMode(PrincipalStep_PIN, OUTPUT);
    pinMode(PrincipalDir_PIN, OUTPUT);
    pinMode(BrazoDir_PIN, OUTPUT);
    pinMode(BrazoStep_PIN, OUTPUT);
    pinMode(VerticalDir_PIN, OUTPUT);
    pinMode(VerticalStep_PIN, OUTPUT);
    pinMode(MotorBomba1, OUTPUT);
    pinMode(MotorBomba2, OUTPUT);
    
    // PID setup
    float gains[3] = {Kp, Ki, Kd};
    dc.setup(Entrada_Motor, Canales_Motor);
    cuadratura.setup(pines_cuadratura, DPA, num_canales, timeout_ms);
    control.setup(gains, dt_ms);
    prev_time = millis();
    
    Serial.println("SCARA Robot - Kinect Mode Ready");
    ConexionBluetooth.println("SCARA Robot - Kinect Mode Ready");
}

void loop() {
    // ==== SECTION I: Serial Data Reception (Kinect/MATLAB) ====
    if (Serial.available()) {
        char buffer[100];
        size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';
        
        // Parse incoming data
        int parsed = sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
                           &AnguloJoint1[0], &AnguloJoint2[0], 
                           &PX[0], &PY[0], &PZ[0],
                           &Proportional[0], &Integral[0], &Derivative[0],
                           &TomaPieza[0]);
        
        if (parsed >= 5) {  // At least position data received
            kinectActive = true;
            lastKinectUpdate = millis();
            
            // Echo to Bluetooth for monitoring
            ConexionBluetooth.printf("RX: %.2f,%.2f,%.2f,%.2f,%.2f\n",
                                    AnguloJoint1[0], AnguloJoint2[0], 
                                    PX[0], PY[0], PZ[0]);
        }
    }
    
    // Check for Kinect timeout
    if (kinectActive && (millis() - lastKinectUpdate > KINECT_TIMEOUT)) {
        kinectActive = false;
        Serial.println("WARN: Kinect connection lost");
        ConexionBluetooth.println("WARN: Kinect connection lost");
    }
    
    // ==== SECTION II: Inverse Kinematics (Position Control) ====
    if (PX[0] != 0 || PY[0] != 0) {
        // Apply smoothing filter for Kinect input
        if (currentMode == MODE_KINECT) {
            smoothX = smoothX * (1 - SMOOTH_FACTOR) + PX[0] * SMOOTH_FACTOR;
            smoothY = smoothY * (1 - SMOOTH_FACTOR) + PY[0] * SMOOTH_FACTOR;
            smoothZ = smoothZ * (1 - SMOOTH_FACTOR) + PZ[0] * SMOOTH_FACTOR;
            
            Posicion[0] = smoothX;
            Posicion[1] = smoothY;
        } else {
            Posicion[0] = PX[0];
            Posicion[1] = PY[0];
        }
        
        // Calculate inverse kinematics
        CinematicaInversa.calculate(Eslabones, Posicion, AngulosSalida);
        
        // Validate angles before moving
        if (isValidAngle(AngulosSalida[0]) && isValidAngle(AngulosSalida[1])) {
            // Move motors
            Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), 
                                      PasosPorRevolucion, 
                                      PrincipalDir_PIN, 0, PrincipalStep_PIN);
            
            Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), 
                              PasosPorRevolucion, 
                              BrazoDir_PIN, Direccion, BrazoStep_PIN);
            
            // Vertical axis (Z control)
            if (PZ[0] != 0) {
                int verticalSteps = mapZToSteps(smoothZ);
                // Implement Z-axis control here
            }
            
            // Feedback
            ConexionBluetooth.printf("IK: PX=%.2f PY=%.2f PZ=%.2f θ1=%.2f θ2=%.2f\n",
                                    Posicion[0], Posicion[1], PZ[0], 
                                    AngulosSalida[0], AngulosSalida[1]);
        } else {
            Serial.println("ERROR: Invalid joint angles - position unreachable");
        }
        
        // Reset for next command
        PX[0] = 0;
        PY[0] = 0;
        PZ[0] = 0;
    }
    
    // ==== SECTION III: Forward Kinematics (Joint Control) ====
    if (AnguloJoint1[0] > 0 && AnguloJoint2[0] > 0) {
        float ParametrosDH[][4] = {
            {AnguloJoint1[0], 30, 0, 0},
            {AnguloJoint2[0], 20, 0, 0}
        };
        
        CinematicaDirecta.setup(ParametrosDH, DoF);
        CinematicaDirecta.calculate(Pose);
        
        Stepper.MotorGiroPrincipal(abs(16 * AnguloJoint1[0]), 
                                  PasosPorRevolucion, 
                                  PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
        
        Stepper.MotorBrazo(abs(16 * AnguloJoint2[0]), 
                          PasosPorRevolucion, 
                          BrazoDir_PIN, Direccion, BrazoStep_PIN);
        
        ConexionBluetooth.printf("FK: α=%.2f β=%.2f γ=%.2f PX=%.2f PY=%.2f PZ=%.2f\n",
                                Pose[0], Pose[1], Pose[2], 
                                Pose[3], Pose[4], Pose[5]);
        
        AnguloJoint1[0] = 0;
        AnguloJoint2[0] = 0;
    }
    
    // ==== SECTION IV: PID Gain Updates ====
    if (Proportional[0] != 0 || Integral[0] != 0 || Derivative[0] != 0) {
        Ganancias[0] = Proportional[0];
        Ganancias[1] = Integral[0];
        Ganancias[2] = Derivative[0];
        control.setGains(Ganancias);
        
        ConexionBluetooth.printf("PID Updated: P=%.3f I=%.3f D=%.3f\n",
                                Ganancias[0], Ganancias[1], Ganancias[2]);
        
        Proportional[0] = 0;
        Integral[0] = 0;
        Derivative[0] = 0;
    }
    
    // ==== SECTION V: Grip/Pick Action ====
    if (TomaPieza[0] != 0) {
        executePickAndPlace();
        TomaPieza[0] = 0;
    }
}

// Helper function: Validate joint angles
bool isValidAngle(float angle) {
    return (angle >= -180.0 && angle <= 180.0);
}

// Helper function: Map Z coordinate to motor steps
int mapZToSteps(float z) {
    // Map Z range (0-30cm) to motor steps
    // Adjust based on your vertical axis configuration
    return (int)((z / 30.0) * 26000);
}

// Execute pick and place sequence
void executePickAndPlace() {
    Serial.println("Executing pick and place...");
    
    // Move to piece location
    Posicion[0] = CoordenadasPieza[0];
    Posicion[1] = CoordenadasPieza[1];
    CinematicaInversa.calculate(Eslabones, Posicion, AngulosSalida);
    
    Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), 
                              PasosPorRevolucion, 
                              PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
    delay(1000);
    
    Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), 
                      PasosPorRevolucion, 
                      BrazoDir_PIN, Direccion, BrazoStep_PIN);
    delay(1000);
    
    // Lower to pick
    Stepper.MotorVertical(26000, PasosPorRevolucion, 
                         VerticalDir_PIN, 0, VerticalStep_PIN);
    
    // Activate gripper/suction
    digitalWrite(MotorBomba1, HIGH);
    digitalWrite(MotorBomba2, LOW);
    delay(3000);
    
    // Raise
    Stepper.MotorVertical(26000, PasosPorRevolucion, 
                         VerticalDir_PIN, 1, VerticalStep_PIN);
    
    // Move to rack position
    CinematicaInversa.calculate(Eslabones, CoordenadasRack_Circulo, AngulosSalida);
    
    Stepper.MotorGiroPrincipal(abs(16 * AngulosSalida[0]), 
                              PasosPorRevolucion, 
                              PrincipalDir_PIN, Direccion, PrincipalStep_PIN);
    delay(1000);
    
    Stepper.MotorBrazo(abs(16 * AngulosSalida[1]), 
                      PasosPorRevolucion, 
                      BrazoDir_PIN, Direccion, BrazoStep_PIN);
    delay(1000);
    
    // Lower to place
    Stepper.MotorVertical(4000, PasosPorRevolucion, 
                         VerticalDir_PIN, 0, VerticalStep_PIN);
    delay(1000);
    
    // Release
    digitalWrite(MotorBomba1, LOW);
    digitalWrite(MotorBomba2, LOW);
    
    Serial.println("Pick and place complete!");
    ConexionBluetooth.println("Pick and place complete!");
}
