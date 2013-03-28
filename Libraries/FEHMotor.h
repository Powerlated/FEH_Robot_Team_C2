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
	void SetPercent( int8 percent );

private:
    FEHMotorPort _motorport;
	char _power;
};

#endif // FEHMOTOR_H
