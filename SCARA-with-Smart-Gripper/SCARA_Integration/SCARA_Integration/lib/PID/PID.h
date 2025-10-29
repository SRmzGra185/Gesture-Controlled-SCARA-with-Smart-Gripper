//Defininicon del índice.
#ifndef __PID_H__
#define __PID_H__


class PID 
{
    public:
        PID(); //Constructor
        ~PID(); //Desconstructor 
        void setup(float gains[3], unsigned int dt_ms); //Void significa que no va a regresar nada, solo es una acción. 
        float calculate(float reference, float measurment); //Programar la retroalimentación y el control.
        void setGains(float gains [3]); //Setear las ganancias.
        float getError();

    private:
        float _gains[3];   //Para saber que es privado el nombre inicia con un: _
        unsigned int _dt_ms;
        float _reference, _error, _saturation;
        float _previousError;
        float _integral;
        
};
#endif // __PID_H__