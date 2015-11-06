#include "FEHMotor.h"
#include "FEHProteus.h"

FEHMotor::FEHMotor( FEHMotorPort motorport, float max_voltage )
{
    _motorport = motorport;
	_power = 0;

    //Votlage Range: 1.0 - 12.0 V
    if(max_voltage > 12)
    {
        _max_voltage = 12.0;
    }
    else if(max_voltage <= 1)
    {
        _max_voltage = 1.0;
    }

    // Set the upper limit of percent power for the motor based on the input max voltage
    // power = 0.6625 * voltage^2 + 0.2396 * voltage [determined experimentally]
    _max_percent = 0.6625 * max_voltage * max_voltage + 0.2396 * max_voltage;
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

void FEHMotor::SetPercent( float percent )
{
    int power;

    //Outside cases
    if (percent < -100)
        percent = -100.;
    else if ( percent > 100 )
        percent = 100.;


    // if percent is positive
    if ( percent >= 0 )
    {
        power = (int) (percent * (127. / _max_percent));
    }
    else
    {
        power = (int) (percent * 128. / _max_percent);
    }

    //Pass to SetPower
    SetPower(power);


}
