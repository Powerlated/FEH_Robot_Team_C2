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

void FEHMotor::SetPower( int power )
{
	int8 power8;
	if(power > 127)
		power8 = 127;
	else if (power< -128)
		power8 = -128;
	else
		power8 = power;
	
	if( _power != power8 )
	{
		_power = power8;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power8, (unsigned char)30 );
	}
}

void FEHMotor::SetPercent( float _percent )
{
	if (_percent < -100)
		_percent = -100.;
	else if (_percent > 100)
		_percent = 100.;
	
	int8 power = ((int) ( _percent  / 100.0 * 127.0 )) & 0xFF;
	
	if( _power != power ) {
		_power = power;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power, (unsigned char)30 );
	}
}
