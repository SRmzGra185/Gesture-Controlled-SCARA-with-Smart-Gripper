#ifndef _HBRIDGE_H
#define _HBRIDGE_H

#include <ESP32_PWM.h>

enum stop
{
    BRAKE,
    COAST
};

class HBridge
{
public:
    HBridge();
    ~HBridge();
    PWM in[2];
    void setup(uint8_t in_pins[2], uint8_t channels[2], float duty_offset = 0, double frequency = 25000.0, uint8_t bits_resolution = 10);
    void setPWMDuty(float duty);
    void setStop(stop mode);
    void setFrequency(float frequency);

private:
    bool _mapping = false;
    float _coeff[2];
};

#endif // _HBRIDGE_H