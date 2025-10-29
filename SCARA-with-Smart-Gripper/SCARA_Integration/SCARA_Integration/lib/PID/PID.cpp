//Definición del constructor.
#include "PID.h"

PID::PID()
{  
}

PID::~PID()
{ 
}

void PID::setup(float gains[3], unsigned int dt_ms)
{
    setGains(gains);
    _dt_ms = dt_ms; //guardar en memoria interna
}

float PID::calculate(float reference, float measurment)
{
    _error = reference - measurment;
    float control_output = _gains[0] * _error;
    if (_gains[1] > 0.0001f)
    {
        _integral += (_dt_ms / 2.0f) * (_error + _previousError);
        //_integral = antiwindup(_integral) ES OPCIONAL AGREGARLO.
        control_output += _gains[1] * _integral;
    }

    if (_gains[2] > 0.0001f)
        control_output += _gains[2] * (_error - _previousError) / _dt_ms;
        _previousError = _error;
        return control_output;
} 

void PID::setGains(float gains[3])
{
    for(char i=0; i < 3; i++)
        _gains[i] = gains[i];
}

float PID::getError()
{
    return _error;
}

