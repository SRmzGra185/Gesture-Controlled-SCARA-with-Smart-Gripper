#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include <Arduino.h>
#include <FunctionalInterrupt.h>

class Encoders
{
    public:
    Encoders();
    ~Encoders();
    void setup(int pin[], float degrees_per_edge, int num_channels, uint16_t timeout_ms);
    float getAngle();
    float getSpeed();
    int8_t getDirection();
       
        
    private:

    void IRAM_ATTR handle();
    uint16_t _timeout_ms;
    int _pin[2];
    int _num_channels;
    float _degrees_per_edge;
    volatile float _speed;
    volatile long _counts;
    volatile ulong _prev_micros, _delta_micros;
    volatile uint8_t _states;
    volatile int8_t _direction;
    const int8_t _single_edge_lut[16] = {0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0};
    const int8_t _double_edge_lut[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        
};



#endif // __ENCODERS_H__
