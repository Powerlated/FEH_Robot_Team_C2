#ifndef FEHMOTOR_H
#define FEHMOTOR_H

#include "derivative.h"

class FEHMotor
{
public:
    typedef enum
    {
        Motor0 = 0,
        Motor1,
        Motor2,
        Motor3
    } FEHMotorPort;

    FEHMotor( FEHMotorPort motorport );

    void Stop();
    void SetPower( int8 power );

private:
    FEHMotorPort _motorport;
};

#endif // FEHMOTOR_H
