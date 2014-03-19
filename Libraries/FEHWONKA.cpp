#include "FEHWONKA.h"
#include "FEHIO.h"
#include "FEHLCD.h"
#include "FEHUtility.h"
#include "FEHServo.h"

FEHWONKA WONKA;

#define OVENMASK 0x07 // first three bits
#define OVENPRESSMASK 0x38 // second three bits
#define CHUTEMASK 0x40 // seventh bit
#define RUNNINGMASK 0x80 // eigth bit

#define STOPDATA 0xAA

void WONKADataProcess( unsigned char *data, unsigned char length );

bool _enabled;
int _region;
float _WONKA_x;
float _WONKA_y;
float _WONKA_heading;
unsigned char _WONKA_objective;
unsigned char _WONKA_time;
bool _WONKA_stop;
bool _WONKA_foundpacket;
FEHServo _irbeacon ( FEHServo::Servo7 );


FEHWONKA::FEHWONKA()
{
    _xbee.SetPacketCallBack( &WONKADataProcess );

	//_enabled = true;
	_region = -1;

    _WONKA_x = 0.0f;
    _WONKA_y = 0.0f;
    _WONKA_heading = 0.0f;
    _WONKA_objective = 0x0;
    _WONKA_time = 0;
    _WONKA_stop = false;
}

void FEHWONKA::InitializeMenu()
{
	ButtonBoard buttons( FEHIO::Bank3 );

	char region = 'A';

	LCD.Clear();
	LCD.WriteLine( "Use LEFT / RIGHT to change region" );
	LCD.WriteLine( "Use MIDDLE to select" );
	LCD.Write( "Region: " );
	LCD.WriteLine( region );

	// wait for user to press middle button
	while( !buttons.MiddlePressed() )
	{
		if( buttons.LeftPressed() )
		{
			region--;
			if( region < 'A' )
			{
				region = 'L';
			}
			LCD.Clear();
			LCD.WriteLine( "Use LEFT / RIGHT to change region" );
			LCD.WriteLine( "Use MIDDLE to select" );
			LCD.Write( "Region: " );
			LCD.WriteLine( region );

			while( buttons.LeftPressed() );
			Sleep( 100 );
		}

		if( buttons.RightPressed() )
		{
			region++;
			if( region > 'L' )
			{
				region = 'A';
			}
			LCD.Clear();
			LCD.WriteLine( "Use LEFT / RIGHT to change region" );
			LCD.WriteLine( "Use MIDDLE to select" );
			LCD.Write( "Region: " );
			LCD.WriteLine( region );

			while( buttons.RightPressed() );
			Sleep( 100 );
		}
	}

	Initialize( region );

	while( buttons.MiddlePressed() );
	Sleep( 1000 );
}

// Manually pick and configure a region
// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
void FEHWONKA::Initialize( int region )
{
	if( !_xbee.IsInitialized() )
		_xbee.Initialize();

	if( !_enabled )
	{
//		_enabled = true;
//		return;

		// check range of region
		if( region < 0 )
		{
			region = 0;
		}
		else if( region > 11 )
		{
			region = 11;
		}

		_region = region;

		// configure xbee

//		_xbee.DisableInterrupt();

		LCD.Clear();

		char txbuffer[ 10 ];
		unsigned int txlength = 0;

		char rxbuffer[ 10 ];
		unsigned int rxlength = 0;

		// enter AT mode
		LCD.Write( "Enter cmd mode..." );
		txbuffer[ 0 ] = '+';
		txbuffer[ 1 ] = '+';
		txbuffer[ 2 ] = '+';
		txlength = _xbee.SendData( txbuffer, 3 );
		//Sleep( 2000 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration. Data: " );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
//			LCD.Write( " " );
		}
		LCD.WriteLine( " " );

		// set my address
		LCD.Write( "Set wireless id..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'M';
		txbuffer[ 3 ] = 'Y';
		txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
		txbuffer[ 5 ] = '1';
		txbuffer[ 6 ] = '0';
		txbuffer[ 7 ] = '0' + ( _region % 4 );
		txbuffer[ 8 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 9 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
//			LCD.Write( " " );
		}
		LCD.WriteLine( " " );

		// set my pan id
		LCD.Write( "Set network id..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'I';
		txbuffer[ 3 ] = 'D';
		txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
		txbuffer[ 5 ] = '0';
		txbuffer[ 6 ] = '0';
		txbuffer[ 7 ] = '0';
		txbuffer[ 8 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 9 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
		}
		LCD.WriteLine( " " );

		// set my destination (low)
		LCD.Write( "Set course id (low)..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'D';
		txbuffer[ 3 ] = 'L';
		txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
		txbuffer[ 5 ] = '0';
		txbuffer[ 6 ] = '0';
		txbuffer[ 7 ] = '0' + ( _region % 4 );
		txbuffer[ 8 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 9 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
		}
		LCD.WriteLine( " " );

		// set my destination (high)
		LCD.Write( "Set course id (high)..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'D';
		txbuffer[ 3 ] = 'H';
		txbuffer[ 4 ] = '0';
		txbuffer[ 5 ] = '0';
		txbuffer[ 6 ] = '0';
		txbuffer[ 7 ] = '0';
		txbuffer[ 8 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 9 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
		}
		LCD.WriteLine( " " );

		// write to flash
		LCD.Write( "Writing config..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'W';
		txbuffer[ 3 ] = 'R';
		txbuffer[ 4 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 5 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
		}
		LCD.WriteLine( " " );

		// exit command mode
		LCD.Write( "Exit command mode..." );
		txbuffer[ 0 ] = 'A';
		txbuffer[ 1 ] = 'T';
		txbuffer[ 2 ] = 'C';
		txbuffer[ 3 ] = 'N';
		txbuffer[ 4 ] = '\r';
		txlength = _xbee.SendData( txbuffer, 5 );
	//	Sleep( 100 );
		rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
		if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
		{
            LCD.WriteLine( "Error with WONKA configuration" );
			return;
		}
		for( int i = 0; i < rxlength; i++ )
		{
			LCD.Write( rxbuffer[ i ] );
		}
		LCD.WriteLine( " " );

		// show success
		LCD.Write( "Successfully initialized  region: " );
		LCD.WriteLine( CurrentRegionLetter() );

//		_xbee.EnableInterrupt();
	}
}

void FEHWONKA::Initialize( char region )
{
	if( region >= 'A' && region <= 'L' )
	{
		Initialize( (int)( region - 'A' ) );
	}
	else if( region >= 'a' && region <= 'l' )
	{
		Initialize( (int)( region - 'a' ) );
	}
}

// Enable receiving of WONKA data
void FEHWONKA::Enable()
{
	_enabled = true;
	_irbeacon.DigitalOn();
	while( WaitForPacket() == 0x00);
	LCD.WriteLine("RPS Enabled Successfully");
}

// Disable receiving of WONKA data
void FEHWONKA::Disable()
{
	_enabled = false;
	_irbeacon.DigitalOff();
}

// return the current course number { 1, 2, 3 }
unsigned char FEHWONKA::CurrentCourse()
{
	if( _region >= 0 )
	{
		return (unsigned char)( _region / 4 + 1 );
	}

	return 0xFF;
}

// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
char FEHWONKA::CurrentRegionLetter()
{
	return ( 'A' + (char)_region );
}

// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
int FEHWONKA::CurrentRegion()
{
	return _region;
}

// Objective functions:

// returns the number of oven button presses required
int FEHWONKA::Oven()
{
    return ( _WONKA_objective & OVENMASK );
}

// returns the number of times the oven button has been pressed
int FEHWONKA::OvenPressed()
{
    return ( ( _WONKA_objective & OVENPRESSMASK ) >> 3 );
}

// returns true if the chute switch has been activated
bool FEHWONKA::Chute()
{
    return ( ( _WONKA_objective & CHUTEMASK ) > 0 );
}

// returns the match time in seconds
unsigned char FEHWONKA::Time()
{
    return _WONKA_time;
}

unsigned char FEHWONKA::WaitForPacket()
{
	unsigned long starttime = TimeNowMSec();
    while( !_WONKA_foundpacket && ( TimeNowMSec() - starttime ) < 1000 );
    if( _WONKA_foundpacket )
	{
        _WONKA_foundpacket = false;
        return _WONKA_time;
	}
	else
	{
		return 0x00;
	}
}

float FEHWONKA::X()
{
	return _WONKA_x;
}

float FEHWONKA::Y()
{
	return _WONKA_y;
}

float FEHWONKA::Heading()
{
	return _WONKA_heading;
}

void WONKADataProcess( unsigned char *data, unsigned char length )
{
	if( _enabled )
	{
		// verify packet length
		//LCD.WriteLine(length);
		if( length == 9 )
		{
			//LCD.WriteLine("HEY I GOT IN THIS FUNKY IF STATEMENT BRO");
            _WONKA_x = (float)( (int)( ( ( (unsigned int)data[ 1 ] ) << 8 ) + (unsigned int)data[ 2 ] ) ) / 10.0f - 1600.0f;
            _WONKA_y = (float)( (int)( ( ( (unsigned int)data[ 3 ] ) << 8 ) + (unsigned int)data[ 4 ] ) ) / 10.0f - 1600.0f;
            _WONKA_heading = (float)( (int)( ( ( (unsigned int)data[ 5 ] ) << 8 ) + (unsigned int)data[ 6 ] ) ) / 10.0f;
            _WONKA_objective = data[ 7 ];
            _WONKA_time = data[ 8 ];
            _WONKA_stop = !(data[7] & RUNNINGMASK);
            _WONKA_foundpacket = true;

//			LCD.Clear();
//			LCD.Write( _WONKA_x );
//			LCD.Write( " " );
//			LCD.Write( _WONKA_y );
//			LCD.Write( " " );
//			LCD.Write( _WONKA_heading );
//			LCD.WriteLine( " " );
//          LCD.Write("Oven Button: ");
//			LCD.WriteLine( WONKA.Oven() );
//          LCD.Write("Presses: ")
//			LCD.WriteLine( WONKA.OvenPresses() );
//			LCD.WriteLine( ( WONKA.Chute() ) ? ( "Chute Closed" ) : ( "Chute Open" ) );
//			LCD.Write( "Time: " );
//			LCD.WriteLine( _WONKA_time );
		}
	}
}
