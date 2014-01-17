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
	float fPercent;

	if(_percent & 0x80) {
		fPercent = -128 + (_percent &0x7f);
	}
	else {
		fPercent = _percent;
	}
	int8 power = ((int) ( fPercent  / 100.0 * 127.0 )) & 0xFF;
	
	if( _power != power ) {
		_power = power;
		Propeller.SetMotorRate( (unsigned char)_motorport, (unsigned char)power, (unsigned char)30 );
	}
}
