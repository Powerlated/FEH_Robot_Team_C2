#include "FEHMotor.h"
#include "FEHProteus.h"

FEHMotor::FEHMotor( FEHMotorPort motorport )
{
    _motorport = motorport;
}

void FEHMotor::Stop()
{
    Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)0, (unsigned char)30 );
}

void FEHMotor::SetPower( int8 power )
{
    Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power, (unsigned char)30 );
}
