#include "HBridge.h"

HBridge::HBridge()
{
}

HBridge::~HBridge()
{
    in[0].~PWM();
    in[1].~PWM();
}

void HBridge::setup(uint8_t in_pins[2], uint8_t channels[2], float duty_offset, double frequency, uint8_t bits_resolution)
{
    in[0].setup(in_pins[0], channels[0], frequency, bits_resolution, HIGH);
    in[1].setup(in_pins[1], channels[1], frequency, bits_resolution, HIGH);
    if (duty_offset > 0.001)
    {
        _mapping = true;
        _coeff[0] = duty_offset;
        _coeff[1] = (100.0f - duty_offset) / 100.0f;
    }
}

void HBridge::setPWMDuty(float duty)
{
    if (_mapping)
        duty = _coeff[0] + duty * _coeff[1];
    if (duty < 0)
    {
        in[0].setDuty(0);
        in[1].setDuty(fabs(duty));
    }
    else
    {
        in[0].setDuty(fabs(duty));
        in[1].setDuty(0);
    }
}

void HBridge::setStop(stop mode)
{
    switch (mode)
    {
    case BRAKE:
        in[0].setDuty(100);
        in[1].setDuty(100);
        break;
    case COAST:
        in[0].setDuty(0);
        in[1].setDuty(0);
        break;
    }
}

void HBridge::setFrequency(float frequency)
{
    in[0].setFrequency(frequency);
    in[1].setFrequency(frequency);
}
