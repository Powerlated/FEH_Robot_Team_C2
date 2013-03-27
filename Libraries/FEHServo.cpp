#include <FEHServo.h>
#include <FEHIO.h>
#include "FEHProteus.h"
#include "derivative.h"
#include "uart.h"
#include "mcg.h"
#include <FEHUtility.h>
#include <FEHLCD.h>



FEHServo::FEHServo( FEHServoPort _servo )
{
    servo_port = _servo;
    servo_min = 500;
    servo_max = 2500;

	// _position == -1 => servo is off
	_position = -1;
}

void FEHServo::SetMin( int _min )
{
    // default to min 500 if value given is outside range
    if( _min < 500 || _min > 2500 )
    {
        servo_min = 500;
    }
    else
    {
        servo_min = _min;
    }
}

void FEHServo::SetMax( int _max )
{
    // default to max 2500 if value given is outside range
    if( _max < 500 || _max > 2500 )
    {
        servo_max = 2500;
    }
    else
    {
        servo_max = _max;
    }
}

void FEHServo::SetDegree( int _degree )
{
	if( _degree < 0 ) _degree = 0;
	if( _degree > 180 ) _degree = 180;

	if( _degree != _position )
	{
		_position = _degree;

		//set rate based on min ,max and degre provided
		unsigned short rate;
		rate = servo_min + (unsigned short)((servo_max - servo_min) / 180.0 * (unsigned short)_degree);

		//to get low byte, typecase to unsigned char
		unsigned char rate_low = (unsigned char)( rate & 0xFF );
		//to get high byte, right shift by eight and then cast
		unsigned char rate_high = (unsigned char)((rate >> 8));

		uart_putchar( UART5_BASE_PTR, 0x7F );
		uart_putchar( UART5_BASE_PTR, 0x05 ); // servo channel set time instruction
		uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port ); // servo to set
		uart_putchar( UART5_BASE_PTR, rate_high ); // on time high byte
		uart_putchar( UART5_BASE_PTR, rate_low ); // on time low byte
		uart_putchar( UART5_BASE_PTR, 0xFF );
	}
}

void FEHServo::Calibrate()
{
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    DigitalInputPin leftbutton( FEHIO::P3_0 );
    DigitalInputPin middlebutton( FEHIO::P3_1 );
    DigitalInputPin rightbutton( FEHIO::P3_2 );

    LCD.WriteLine( "Use left and right buttons" );
    LCD.WriteLine( "to select min." );
    LCD.WriteLine( "Press middle button" );
    LCD.WriteLine( "when complete." );
    // set servo to 0 degrees using default min
    while( middlebutton.Value() )
    {
        this->SetDegree( 0 );
        while( leftbutton.Value() && rightbutton.Value() && middlebutton.Value() );

        // set min using left and right buttons
        if ( !leftbutton.Value() )
        {
            servo_min = servo_min - 1;
            if( servo_min < 500 ) servo_min = 500;
        }
        if ( !rightbutton.Value() )
        {
            servo_min = servo_min + 1;
            if( servo_min > 2500 ) servo_min = 2500;
        }
        Sleep( 5 );
    }
    while(!middlebutton.Value());
    Sleep(500);

    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine( "Use left and right buttons" );
    LCD.WriteLine( "to select max." );
    LCD.WriteLine( "Press middle button" );
    LCD.WriteLine( "when complete." );
    // set servo to 180 using default max
    while( middlebutton.Value() )
    {
        this->SetDegree( 180 );
        while( leftbutton.Value() && rightbutton.Value() && middlebutton.Value() );

        // set max using left and right buttons
        if ( !leftbutton.Value() )
        {
            servo_max = servo_max - 1;
            if( servo_max < 500 ) servo_max = 500;
        }
        if ( !rightbutton.Value() )
        {
            servo_max = servo_max + 1;
            if( servo_max > 2500 ) servo_max = 2500;
        }
        Sleep( 5 );
    }

    // Print out servo min and servo max
    LCD.Clear( FEHLCD::Black );
    LCD.Write( "Min = " );
    LCD.Write( servo_min );
    LCD.Write( "   Max = " );
    LCD.WriteLine( servo_max );

}

void FEHServo::Off()
{
	if( _position >= 0 )
	{
		_position = -1;

		uart_putchar( UART5_BASE_PTR, 0x7F );
		uart_putchar( UART5_BASE_PTR, 0x06 ); // servo off type
		uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port ); // servo to turn off
		uart_putchar( UART5_BASE_PTR, 0xFF );
	}
}
