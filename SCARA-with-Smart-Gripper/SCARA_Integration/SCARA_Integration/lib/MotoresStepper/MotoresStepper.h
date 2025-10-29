#ifndef __MOTORESSTEPPER_H__
#define __MOTORESSTEPPER_H__

#include<Arduino.h>

class MotoresStepper
{
    public:
    MotoresStepper();
    ~MotoresStepper();
    void setup(uint8_t PrincipalStep_PIN, uint8_t PrincipalDir_PIN, uint8_t BrazoDir_PIN, uint8_t BrazoStep_PIN, uint8_t VerticalDir_PIN, uint8_t VerticalStep_PIN);
    void MotorVertical(int degrees, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN, int direction, uint8_t STEP_PIN);
    void MotorGiroPrincipal(int degrees1, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN1, int direction1, uint8_t STEP_PIN1);
    void MotorBrazo(int degrees2, float STEPS_PER_REVOLUTION, uint8_t DIR_PIN2, int direction2, uint8_t STEP_PIN2);
    void Home();
    
    private:
    protected:
};
#endif // __MOTORESSTEPPER_H__