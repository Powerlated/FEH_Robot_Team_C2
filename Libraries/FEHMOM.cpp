#include "FEHMOM.h"
#include "FEHIO.h"
#include "FEHLCD.h"
#include "FEHUtility.h"

FEHMOM MOM;

#define STONEMASK 0x01
#define GENERATORMASK 0x02
#define TOPBUTTONMASK 0x04
#define BOTTOMBUTTONMASK 0x8

#define STOPDATA 0xAA

void MOMDataProcess( unsigned char *data, unsigned char length );

bool _enabled;
int _region;

float _mom_x;
float _mom_y;
float _mom_heading;
unsigned char _mom_objective;
unsigned char _mom_time;
bool _mom_stop;
bool _mom_foundpacket;

FEHMOM::FEHMOM()
{
	_xbee.SetPacketCallBack( &MOMDataProcess );

	//_enabled = true;
	_region = -1;

	_mom_x = 0.0f;
	_mom_y = 0.0f;
	_mom_heading = 0.0f;
	_mom_objective = 0x0;
	_mom_time = 0;
	_mom_stop = false;
}

void FEHMOM::InitializeMenu()
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
	Sleep( 100 );
}

// Manually pick and configure a region
// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
void FEHMOM::Initialize( int region )
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
			LCD.WriteLine( "Error with MOM configuration. Data: " );
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
			LCD.WriteLine( "Error with MOM configuration" );
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
			LCD.WriteLine( "Error with MOM configuration" );
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
			LCD.WriteLine( "Error with MOM configuration" );
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
			LCD.WriteLine( "Error with MOM configuration" );
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
			LCD.WriteLine( "Error with MOM configuration" );
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
			LCD.WriteLine( "Error with MOM configuration" );
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

void FEHMOM::Initialize( char region )
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

// Enable receiving of MOM data
void FEHMOM::Enable()
{
	_enabled = true;
}

// Disable receiving of MOM data
void FEHMOM::Disable()
{
	_enabled = false;
}

// return the current course number { 1, 2, 3 }
unsigned char FEHMOM::CurrentCourse()
{
	if( _region >= 0 )
	{
		return (unsigned char)( _region / 4 + 1 );
	}

	return 0xFF;
}

// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
char FEHMOM::CurrentRegionLetter()
{
	return ( 'A' + (char)_region );
}

// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
int FEHMOM::CurrentRegion()
{
	return _region;
}

// Objective functions:

// returns true if the RIGHT Stone should be moved
bool FEHMOM::Stone()
{
	return ( ( _mom_objective & STONEMASK ) > 0 );
}

// returns true if Generator switch should be moved BACKWARD
bool FEHMOM::Generator()
{
	return ( ( _mom_objective & GENERATORMASK ) > 0 );
}

// returns true if Top Satellite button has been pressed
bool FEHMOM::TopButton()
{
	return ( ( _mom_objective & TOPBUTTONMASK ) > 0 );
}

// returns true if Bottom Satellite button has been pressed
bool FEHMOM::BottomButton()
{
	return ( ( _mom_objective & BOTTOMBUTTONMASK ) > 0 );
}

// returns the match time in seconds
unsigned char FEHMOM::Time()
{
	return _mom_time;
}

unsigned char FEHMOM::WaitForPacket()
{
	unsigned long starttime = TimeNowMSec();
	while( !_mom_foundpacket && ( TimeNowMSec() - starttime ) < 1000 );
	if( _mom_foundpacket )
	{
		_mom_foundpacket = false;
		return _mom_time;
	}
	else
	{
		return 0x00;
	}
}

void MOMDataProcess( unsigned char *data, unsigned char length )
{
	if( _enabled )
	{
		// verify packet length
		if( length == 9 )
		{
			_mom_x = (float)( (int)( ( ( (unsigned int)data[ 1 ] ) << 8 ) + (unsigned int)data[ 2 ] ) ) / 10.0f - 72.0f;
			_mom_y = (float)( (int)( ( ( (unsigned int)data[ 3 ] ) << 8 ) + (unsigned int)data[ 4 ] ) ) / 10.0f;
			_mom_heading = (float)( (int)( ( ( (unsigned int)data[ 5 ] ) << 8 ) + (unsigned int)data[ 6 ] ) ) / 10.0f;
			_mom_objective = data[ 7 ];
			_mom_time = data[ 8 ];
			_mom_stop = ( _mom_time == STOPDATA );

			_mom_foundpacket = true;

//			LCD.Clear();
//			LCD.Write( _mom_x );
//			LCD.Write( " " );
//			LCD.Write( _mom_y );
//			LCD.Write( " " );
//			LCD.Write( _mom_heading );
//			LCD.WriteLine( " " );
//			LCD.WriteLine( ( MOM.Stone() ) ? ( "RIGHT stone" ) : ( "LEFT stone" ) );
//			LCD.WriteLine( ( MOM.Generator() ) ? ( "Generator backward" ) : ( "Generator forward" ) );
//			LCD.WriteLine( ( MOM.TopButton() ) ? ( "Top pressed" ) : ( "Top released" ) );
//			LCD.WriteLine( ( MOM.BottomButton() ) ? ( "Bottom pressed" ) : ( "Bottom released" ) );
//			LCD.Write( "Time: " );
//			LCD.WriteLine( _mom_time );
		}
	}
}
