#include "FEHRCS.h"
#include "FEHIO.h"
#include "FEHLCD.h"
#include "FEHSD.h"
#include "FEHUtility.h"
#include "FEHServo.h"
#include "string.h"

FEHRCS RCS;

#define STOPDATA 0xAA
#define REGION_COUNT 4

void RCSDataProcess( unsigned char *data, unsigned char length );

bool _enabled;
int _region;
float _RCS_x;
float _RCS_y;
float _RCS_heading;
long _RCS_objective;
unsigned char _RCS_time;
bool _RCS_stop;
bool _RCS_foundpacket;


FEHRCS::FEHRCS()
{
    _xbee.SetPacketCallBack( &RCSDataProcess );

	//_enabled = true;
	_region = -1;

    _RCS_x = 0.0f;
    _RCS_y = 0.0f;
    _RCS_heading = 0.0f;
    _RCS_objective = 0x0;
    _RCS_time = 0;
    _RCS_stop = false;
}

// Manually pick and configure a region
// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
void FEHRCS::Initialize( int region, const char* team_key )
{
    if( !_xbee.IsInitialized() )
        _xbee.Initialize();

    if( !_enabled )
    {
//		if( !_xbee.IsInitialized() )
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

        //_xbee.DisableInterrupt();

        LCD.Clear();

        char txbuffer[ 10 ];
        unsigned int txlength = 0;

        char rxbuffer[ 10 ];
        unsigned int rxlength = 0;

        // enter AT mode
        bool entered_cmd_mode(false);
        while(!entered_cmd_mode)
        {
            LCD.Write( "Enter cmd mode..." );
            txbuffer[ 0 ] = '+';
            txbuffer[ 1 ] = '+';
            txbuffer[ 2 ] = '+';
            txlength = _xbee.SendData( txbuffer, 3 );
            //Sleep( 1000 );
            rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
            if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
            {
                LCD.WriteLine(" ");
                LCD.WriteLine("Waiting for response...");
                Sleep(2500);
                rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, '0');
                if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
                {
                    LCD.WriteLine( "Enter cmd mode FAILED" );
                    LCD.Write( "with Data: ");
                    for(int i=0 ; i<rxlength ; i++)
                    {
                        LCD.Write(rxbuffer[i]);
                    }
                    LCD.WriteLine( " " );
                    LCD.WriteLine("Trying again...");
                }
                else
                    entered_cmd_mode = true;
            }
            else
                entered_cmd_mode = true;
        }
        LCD.Write("Data: ");
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
            LCD.WriteLine( "Error with RCS configuration" );
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
            LCD.WriteLine( "Error with RCS configuration" );
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
            LCD.WriteLine( "Error with RCS configuration" );
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
            LCD.WriteLine( "Error with RCS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );
        
         // set my destination (high)
        LCD.Write( "Set channel 26 2480MHz" );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'C';
        txbuffer[ 3 ] = 'H';
        txbuffer[ 4 ] = '0';
        txbuffer[ 5 ] = '0';
        txbuffer[ 6 ] = '1';
        txbuffer[ 7 ] = 'A';
        txbuffer[ 8 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 9 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RCS configuration" );
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
            LCD.WriteLine( "Error with RCS configuration" );
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
            LCD.WriteLine( "Error with RCS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // Sending team identifier string to RCS computer
        LCD.WriteLine("Team Key Sent: ");
        for (int i = 0; i < 9; i++) {
            // Remember to write some sort of error checking
            txbuffer[i] = *(team_key + i);
            LCD.Write(*(team_key + i));
        }
        LCD.WriteLine(" ");
        txbuffer[9] = '\r';
        txlength = _xbee.SendData( txbuffer, 10 );

        // show success
        LCD.Write( "Connecting to Region " );
        LCD.WriteLine( CurrentRegionLetter() );

//		_xbee.EnableInterrupt();

        //Wait to ensure that a connection is made
        _enabled = true;
        int MAX_CONNECT_TRIES = 5;
        int i = 0;
        while( WaitForPacket() == 0x00) {
            if (i > MAX_CONNECT_TRIES) {
                LCD.Clear();
                LCD.Write("RCS unable to connect to region ");
                LCD.WriteLine((char)region);

                LCD.WriteLine("Make sure no other Protei are connected to the region");
                LCD.WriteLine("Restart the region and retry connecting with your Proteus!");
                while(true);
            }
            txlength = _xbee.SendData( txbuffer, 10 );
        }
        LCD.WriteLine("RCS Enabled Successfully, Program running in 3 seconds");
        Sleep(3000);
        LCD.Clear();
    }
    }
}

void FEHRCS::Initialize( char region, const char* team_key )
{
    if( region >= 'A' && region <= 'L' )
    {
        Initialize( (int)( region - 'A' ), team_key );
    }
    else if( region >= 'a' && region <= 'l' )
    {
        Initialize( (int)( region - 'a' ), team_key );
    }
}

void FEHRCS::InitializeMenu( const char* team_key )
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

	Initialize( region, team_key );

	//while( buttons.MiddlePressed() );
	//Sleep( 1000 );
}

void FEHRCS::InitializeTouchMenu(const char* team_key)
{
	int cancel = 1;
    int c=0, d=0, n;
    float x, y;
    char region;

    FEHIcon::Icon regions_title[1];
    char regions_title_label[1][20] = {"Select RCS Region"};

    FEHIcon::Icon regions[REGION_COUNT];
    char regions_labels[12][20] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

    FEHIcon::Icon confirm_title[1];
    char confirm_title_label[1][20] = {""};

    FEHIcon::Icon confirm[2];
    char confirm_labels[2][20] = {"Ok", "Cancel"};

	while(cancel) {
    c=0;
    d=0;
    LCD.Clear(BLACK);

    int regionLabelRowCount = REGION_COUNT / 4;

    FEHIcon::DrawIconArray(regions_title, 1, 1, 1, 201, 1, 1, regions_title_label, BLACK, WHITE);
    FEHIcon::DrawIconArray(regions, regionLabelRowCount, 4, 40, 2, 1, 1, regions_labels, WHITE, WHITE);

	while (!c)
	{
        if (LCD.Touch(&x, &y))
        {
            for (n=0; n<=7; n++)
            {
                if (regions[n].Pressed(x, y, 0))
                {
                    regions[n].WhilePressed(x, y);
                    c = n+1;
                }
            }
        }
	}
    switch (c)
    {
    case 1:
        region = 'A';
        strcpy(confirm_title_label[0], "Choice: A");
        break;
    case 2:
        region = 'B';
        strcpy(confirm_title_label[0], "Choice: B");
        break;
    case 3:
        region = 'C';
        strcpy(confirm_title_label[0], "Choice: C");
        break;
    case 4:
        region = 'D';
        strcpy(confirm_title_label[0], "Choice: D");
        break;
    case 5:
        region = 'E';
        strcpy(confirm_title_label[0], "Choice: E");
        break;
    case 6:
        region = 'F';
        strcpy(confirm_title_label[0], "Choice: F");
        break;
    case 7:
        region = 'G';
        strcpy(confirm_title_label[0], "Choice: G");
        break;
    case 8:
        region = 'H';
        strcpy(confirm_title_label[0], "Choice: H");
        break;
    }

    LCD.Clear(BLACK);
    FEHIcon::DrawIconArray(confirm_title, 1, 1, 60, 201, 1, 1, confirm_title_label, BLACK, WHITE);
    FEHIcon::DrawIconArray(confirm, 1, 2, 60, 60, 1, 1, confirm_labels, WHITE, WHITE);

	while (!d)
	{
        if (LCD.Touch(&x, &y))
        {
            for (n=0; n<=1; n++)
            {
                if (confirm[n].Pressed(x, y, 0))
                {
                    confirm[n].WhilePressed(x, y);
                    d = n+1;
                }
            }
        }
	}
    switch (d)
    {
    case 1:
        cancel = 0;
        break;
    case 2:
        cancel = 1;
        break;
    }
    }

    Initialize( region , team_key );

}


// return the current course number { 1, 2, 3 }
unsigned char FEHRCS::CurrentCourse()
{
	if( _region >= 0 )
	{
		return (unsigned char)( _region / 4 + 1 );
	}

	return 0xFF;
}

// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
char FEHRCS::CurrentRegionLetter()
{
	return ( 'A' + (char)_region );
}

// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
int FEHRCS::CurrentRegion()
{
	return _region;
}

// Objective function
int FEHRCS::GetCorrectLever()
{
    return (int)_RCS_objective;
}

// returns the match time in seconds
int FEHRCS::Time()
{
    return (int)_RCS_time;
}

unsigned char FEHRCS::WaitForPacket()
{
	unsigned long starttime = TimeNowMSec();
    while( !_RCS_foundpacket && ( TimeNowMSec() - starttime ) < 1000 );
    if( _RCS_foundpacket )
	{
        _RCS_foundpacket = false;
        return 0xFF;
	}
	else
	{
		return 0x00;
	}
}

int FEHRCS::WaitForPacketDebug(int *packetsFound, int *packetsLost, int *lastFoundPacketTime)
{
  Sleep(100);
  //If packets are found, increment the number of packets
  if(_RCS_foundpacket){
    int packetFoundEndTime = TimeNowMSec() - *lastFoundPacketTime;
    if(*lastFoundPacketTime != 0){
      *packetsFound = packetFoundEndTime/10;
    }

    //LCD.WriteLine((*packetsFound));
    *lastFoundPacketTime = TimeNowMSec();
    //LCD.WriteLine("Booger");


  }



  //If packets aren't lost, we don't want to report that they're lost
  int packetLossStartTime = TimeNowMSec();
  int packetLossEndTime = packetLossStartTime;

  while(!_RCS_foundpacket){
    //However if packets are being lost, we want to figure out how long we're losing them for
    packetLossEndTime = TimeNowMSec();
  }
  //Always assume RCS is broken (this will quickly be set to true by RCSDataProcess)
  _RCS_foundpacket = false;

  //A packet has now been found, so stop the clock
  int endtime = TimeNowMSec();

  //This is based on a baud rate of 9600 #theyDidTheMath
  *packetsLost = (packetLossEndTime-packetLossStartTime)/10;
   int elapsedtime = packetLossEndTime-packetLossStartTime;


  return elapsedtime;


}

// float FEHRCS::X()
// {
// 	return _RCS_x;
// }

// float FEHRCS::Y()
// {
// 	return _RCS_y;
// }

// float FEHRCS::Heading()
// {
// 	return _RCS_heading;
// }

void RCSDataProcess( unsigned char *data, unsigned char length )
{
	if( _enabled )
	{
        /*
            New packet structure:
            [0] : RCS Start Byte
            [1 - 4] : RCS Objective Data
            [5] : RCS Stop
        */

        // _RCS_x = (float)( (int)( ( ( (unsigned int)data[ 1 ] ) << 8 ) + (unsigned int)data[ 2 ] ) ) / 10.0f - 1600.0f;
        // _RCS_y = (float)( (int)( ( ( (unsigned int)data[ 3 ] ) << 8 ) + (unsigned int)data[ 4 ] ) ) / 10.0f - 1600.0f;
        // _RCS_heading = (float)( (int)( ( ( (unsigned int)data[ 5 ] ) << 8 ) + (unsigned int)data[ 6 ] ) ) / 10.0f - 1600.0f;
		_RCS_objective = (((unsigned int)(data[ 1 ]) << 24) | ((unsigned int)(data[ 2 ]) << 16) | ((unsigned int)(data[ 3 ]) << 8) | (data[ 4 ]));
        _RCS_time = data[ 5 ];
        _RCS_stop = (data[ 6 ] == STOPDATA);
        _RCS_foundpacket = true;

        if(_RCS_stop)
		{
			// set kill pin low for power shutdown
            SD.FCloseAll();
			GPIOD_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( 13 ) );
		}
	}


}
