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
}

void FEHMOM::InitializeMenu()
{
	ButtonBoard buttons( FEHIO::Bank3 );

	char region = 'A';
	bool usesurrogate = true;

	LCD.Clear();
	LCD.WriteLine( "Use LEFT / RIGHT to change option" );
	LCD.WriteLine( "Use MIDDLE to select" );

	LCD.Write( "Use Surrogate MOM? " );
	if( usesurrogate )
	{
		LCD.WriteLine( "YES" );
	}
	else
	{
		LCD.WriteLine( "NO" );
	}

	// wait for user to press middle button
	while( !buttons.MiddlePressed() )
	{
		if( buttons.LeftPressed() || buttons.RightPressed() )
		{
			usesurrogate = !usesurrogate;
			LCD.Clear();
			LCD.WriteLine( "Use LEFT / RIGHT to change option" );
			LCD.WriteLine( "Use MIDDLE to select" );

			LCD.Write( "Use Surrogate MOM? " );
			if( usesurrogate )
			{
				LCD.WriteLine( "YES" );
			}
			else
			{
				LCD.WriteLine( "NO" );
			}
			while( buttons.LeftPressed() || buttons.RightPressed() );
			Sleep( 100 );
		}
	}

	if( !usesurrogate )
	{
		LCD.WriteLine( " " );
		LCD.WriteLine( "MOM is currently busy" );
		LCD.WriteLine( "She will be back soon" );
		LCD.WriteLine( "Until then, Surrogate MOM" );
		LCD.WriteLine( " will be help you" );
		Sleep( 5000 );
	}

	LCD.Clear();
	LCD.WriteLine( "Use LEFT / RIGHT to change region" );
	LCD.WriteLine( "Use MIDDLE to select" );

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
    while(buttons.MiddlePressed());
    Sleep(100);

    Initialize( region );


    bool stones = false;
    bool generator = false;

    LCD.Clear();
    LCD.WriteLine( "Use LEFT to change STONES," );
    LCD.WriteLine( "RIGHT to change GENERATOR" );
    LCD.WriteLine( "Use MIDDLE to select" );
    LCD.WriteLine(" ");

    LCD.Write( "STONES: " );
    LCD.WriteLine( (stones ? "Right" : "Left" ));

    LCD.Write( "GENERATOR: " );
    LCD.WriteLine( (generator ? "Backward" : "Foreward" ));;

    while( !buttons.MiddlePressed() )
    {
        if( buttons.LeftPressed() )
        {
            stones = ! stones;

            LCD.Clear();
            LCD.WriteLine( "Use LEFT to change STONES," );
            LCD.WriteLine( "RIGHT to change GENERATOR" );
            LCD.WriteLine( "Use MIDDLE to select" );

            LCD.WriteLine(" ");
            LCD.Write( "STONES: " );
            LCD.WriteLine( (stones ? "Right" : "Left" ));


            LCD.Write( "GENERATOR: " );
            LCD.WriteLine( (generator ? "Backward" : "Foreward" ));

            while( buttons.LeftPressed() );
            Sleep( 100 );
        }

        if( buttons.RightPressed() )
        {
            generator = !generator;

            LCD.Clear();
            LCD.WriteLine( "Use LEFT to change STONES," );
            LCD.WriteLine( "RIGHT to change GENERATOR" );
            LCD.WriteLine( "Use MIDDLE to select" );

            LCD.WriteLine(" ");
            LCD.Write( "STONES: " );
            LCD.WriteLine( (stones ? "Right" : "Left" ));


            LCD.Write( "GENERATOR: " );
            LCD.WriteLine( (generator ? "Backward" : "Foreward" ));

            while( buttons.RightPressed() );
            Sleep( 100 );
        }
    }

	while( buttons.MiddlePressed() );
	Sleep( 100 );
    _mom_objective = (STONEMASK * ((int) stones)) | (GENERATORMASK * ((int) generator));
    LCD.WriteLine(" ");
    //LCD.Write(_mom_objective);
}

// Manually pick and configure a region
// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
void FEHMOM::Initialize( int region )
{
    _region = region;
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

void MOMDataProcess( unsigned char *data, unsigned char length )
{
}
