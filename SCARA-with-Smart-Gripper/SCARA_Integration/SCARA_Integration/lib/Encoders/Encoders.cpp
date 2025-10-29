#include "Encoders.h"

Encoders::Encoders()
{
}

Encoders::~Encoders()
{
}

void Encoders::setup(int pin[], float degrees_per_edge, int num_channels, uint16_t timeout_ms)
{

    _timeout_ms = timeout_ms;
    _degrees_per_edge = degrees_per_edge;
    _num_channels = num_channels;
    for (uint8_t i = 0; i < 2; i++)
    {
        _pin[i] = pin[i];
        pinMode(_pin[i], INPUT);
        bitWrite(_states, i, digitalRead(_pin[i]));
    }
    attachInterrupt(_pin[0], std::bind(&Encoders::handle, this), CHANGE);
    if (_num_channels > 1)
        attachInterrupt(_pin[1], std::bind(&Encoders::handle, this), CHANGE);
    _delta_micros = _timeout_ms * 1000;
    _prev_micros = micros();
}

void IRAM_ATTR Encoders::handle()
{
    volatile ulong current_micros = micros();
    _delta_micros = current_micros - _prev_micros;
    _prev_micros = current_micros;
    _states = _states << 2;
    for (volatile uint8_t i = 0; i < 2; i++)
        bitWrite(_states, i, digitalRead(_pin[i]));
    if (_num_channels > 1)
        _direction = _double_edge_lut[_states & 0b1111];
    else
        _direction = _single_edge_lut[_states & 0b1111];
    _counts += _direction;
}

float Encoders::getAngle()
{
    return _counts * _degrees_per_edge;
}

float Encoders::getSpeed()
{
    ulong current_micros = micros();
    if ((current_micros - _prev_micros) / 1000 < _timeout_ms)
        _speed = _direction * (1000000 * _degrees_per_edge) / _delta_micros;
    else
        _speed = 0.0f;
    return _speed;
}

int8_t Encoders::getDirection()
{
    return _direction;
}


