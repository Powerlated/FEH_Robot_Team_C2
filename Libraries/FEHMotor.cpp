#include "FEHMotor.h"
#include "FEHProteus.h"

FEHMotor::FEHMotor( FEHMotorPort motorport )
{
    _motorport = motorport;
	_power = 0;
}

void FEHMotor::Stop()
{
	if( _power != 0 )
	{
		_power = 0;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)0, (unsigned char)30 );
	}
}

void FEHMotor::SetPower( int8 power )
{
	if( _power != power )
	{
		_power = power;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power, (unsigned char)30 );
	}
}

void FEHMotor::SetPercent( int8 _percent )
{
	int8 power = (int8)( ( (float)_percent ) / 100.0 * 127.0 );
	if( _power != power )
	{
		_power = power;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power, (unsigned char)30 );
	}
}
