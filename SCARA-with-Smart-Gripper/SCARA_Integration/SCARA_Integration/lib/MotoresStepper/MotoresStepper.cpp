#include <MotoresStepper.h>

MotoresStepper::MotoresStepper()
{
}

MotoresStepper::~MotoresStepper()
{
}

void MotoresStepper::setup(uint8_t PrincipalStep_PIN, uint8_t PrincipalDir_PIN, uint8_t BrazoDir_PIN, uint8_t BrazoStep_PIN, uint8_t VerticalDir_PIN, uint8_t VerticalStep_PIN)
{
  pinMode(PrincipalStep_PIN, OUTPUT);
  pinMode(PrincipalDir_PIN, OUTPUT);
  pinMode(BrazoDir_PIN, OUTPUT);
  pinMode(BrazoStep_PIN, OUTPUT);
  pinMode(VerticalDir_PIN, OUTPUT);
  pinMode(VerticalStep_PIN, OUTPUT);
}

void MotoresStepper::MotorVertical(int degrees, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN, int direction, uint8_t STEP_PIN)
{
  // Calculate the number of steps required to rotate the specified degrees
  int steps = (degrees / 360.0) * STEPS_PER_REVOLUTION;

  // Set the direction pin according to the specified direction
  digitalWrite(DIR_PIN, direction);

  // Step the motor the specified number of steps
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
  }
}

void MotoresStepper::MotorGiroPrincipal(int degrees1, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN1, int direction1, uint8_t STEP_PIN1)
{
  // Calculate the number of steps required to rotate the specified degrees
  int steps = (degrees1 / 360.0) * STEPS_PER_REVOLUTION;

  // Set the direction pin according to the specified direction
  digitalWrite(DIR_PIN1, direction1);

  // Step the motor the specified number of steps
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
    digitalWrite(STEP_PIN1, LOW);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
    
  }
}

void MotoresStepper::MotorBrazo(int degrees2, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN2, int direction2, uint8_t STEP_PIN2)
{
  // Calculate the number of steps required to rotate the specified degrees
  int steps = (degrees2 / 360.0) * STEPS_PER_REVOLUTION;

  // Set the direction pin according to the specified direction
  digitalWrite(DIR_PIN2, direction2);

  // Step the motor the specified number of steps
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(800); // Adjust the delay as per your motor's requirement
  }
}

void MotoresStepper::Home()
{
  
}
