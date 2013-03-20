#include "FEHLCD.h"
#include "FEHUtility.h"

#include "derivative.h"
#include "mcg.h"
#include "lptmr.h"

#define CLR_CS GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_CS GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )

#define CLR_RS GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 18 ) )
#define SET_RS GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 18 ) )

#define CLR_Rd GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 16 ) )
#define SET_RD GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 16 ) )

#define CLR_WR GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 17 ) )
#define SET_WR GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 17 ) )

#define CLR_RESET GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_RESET GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )

// font height = 20, maxlines = 240 / 20
// TODO: support portrait text
#define MAXLINES 12

FEHLCD LCD;

bool initialized = false;

FEHLCD::FEHLCD()
{
    _forecolor = White;
    _backcolor = Black;

    _currentline = 0;
    _currentchar = 0;
}

void FEHLCD::Initialize()
{
    if( initialized == false )
    {
        initialized = true;

        _Initialize();
    }
}

void FEHLCD::Clear( FEHLCDColor color )
{
    _backcolor = color;

    // TODO: Clear screen
    _Clear();
}

void FEHLCD::SetFontColor( FEHLCDColor color )
{
    _forecolor = color;
}

void FEHLCD::Write( const char *str )
{

}

//void FEHLCD::Write( string str )
//{

//}



void FEHLCD::Write( int i )
{
//    string s;
//    s += i;
//    Write( s );
}

void FEHLCD::Write( float f )
{
//    string s;
//    s += f;
//    Write( s );
}

void FEHLCD::Write( double d )
{
//    string s;
//    s += d;
//    Write( s );
}

void FEHLCD::Write( bool b )
{
    if( b )
    {
        Write( "true" );
    }
    else
    {
        Write( "false" );
    }
}

void FEHLCD::WriteLine( const char* str )
{
    Write( str );
    NextLine();
}

//void FEHLCD::WriteLine( string str )
//{
//    Write( str );
//    NextLine();
//}

void FEHLCD::WriteLine( int i )
{
    Write( i );
    NextLine();
}

void FEHLCD::WriteLine( float f )
{
    Write( f );
    NextLine();
}

void FEHLCD::WriteLine( double d )
{
    Write( d );
    NextLine();
}

void FEHLCD::WriteLine( bool b )
{
    Write( b );
    NextLine();
}

void FEHLCD::_Initialize()
{
    // Setup pins
    // CS - C3
    // RS - B18
    // RD - B16
    // WR - B17
    // RESET - B3
    PORTC_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR3 = ( PORT_PCR_MUX( 1 ) );
    GPIOC_PDDR |= ( 1 << 3 );
    GPIOB_PDDR |= ( ( 1 << 18 ) );
    GPIOB_PDDR |= ( ( 1 << 16 ) );
    GPIOB_PDDR |= ( ( 1 << 17 ) );
    SET_RESET;
    GPIOB_PDDR |= ( ( 1 << 3 ) );

    // D0 - B9
    // D1 - B8
    // D2 - C4
    // D3 - C5
    // D4 - C6
    // D5 - C7
    // D6 - C12
    // D7 - C13
    // D8 - C14
    // D9 - C15
    // D10 - C16
    // D11 - C17
    // D12 - C18
    // D13 - C19
    // D14 - D0
    // D15 - D2
    // D16 - D3
    // D17 - D4
    PORTB_PCR9 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR8 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR4 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR5 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR6 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR7 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR12 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR13 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR14 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR15 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR19 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR0 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR2 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR4 = ( PORT_PCR_MUX( 1 ) );
    GPIOB_PDDR |= ( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDDR |= ( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDDR |= ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );

    // CS = 1;
    SET_CS;

    // RESET = 1;
    SET_RESET;

    Sleep( 500 );

    // uchar i;

    // widx(0x36); //set adress mode
    WriteIndex( 0x36 );

    // wpat( 0x00 ); //set page,column,line,RGB adress order;display data latch data,fli horizontal,vertical
    WriteParameter( 0x00 );

    // widx( 0xf0 ); //set pixel data interface
    WriteIndex( 0xF0 );

    // wpat( 0x04 ); //select 18bit
    WriteParameter( 0x04 );

    // widx( 0x29 );  //display on
    WriteIndex( 0x29 );
}

void FEHLCD::WriteIndex( unsigned char index )
{
    // CS = 0;
    CLR_CS;

    // RS = 0;
    CLR_RS;

    // RD = 1;
    SET_RD;

    // dataport0 = index;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 )

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // // dataport0 = 0xff;
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // RS = 1;
    SET_RS;

    // delay2( 1 );
    Delay2( 1 );
}

void FEHLCD::WriteParameter( unsigned char param )
{
    // CS = 0;
    CLR_CS;

    // RD = 1;
    SET_RD;

    // RS = 1;
    SET_RS;

    // dataport0 = data1;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 );

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // //dataport0 = 0xff;

    // RS = 0;
    CLR_RS;

    // delay2( 1 );
    Delay2( 1 );
}

void FEHLCD::Delay2( unsigned int t )
{
    unsigned int i;
    unsigned int j;
    for( i = 0; i < t; i++ );
}

void FEHLCD::_Clear()
{
    unsigned int color = 0x00000;

    switch( _backcolor )
    {
        case Black: color = 0x00000; break;
        case White: color = 0x3FFFF; break;
    }

    unsigned char c1 = (unsigned char)( color & 0xFF );
    unsigned char c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    unsigned char c3 = (unsigned char)( ( color >> 16 ) & 0x03 );

    // widx(0x2a); //set column address
    WriteIndex( 0x2A );

    // wpat(0x00); //start column number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start column number low byte
    WriteParameter( 0x00 );

    //wpat(0x01); //end column number high byte
    WriteParameter( 0x01 );

    //wpat(0x3f); //end column number low byte
    WriteParameter( 0x3F );

    //widx(0x2b); //set row adress
    WriteIndex( 0x2b );

    //wpat(0x00); //start row number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start row number low byte
    WriteParameter( 0x00 );

    //wpat(0x00); //end row number high byte
    WriteParameter( 0x00 );

    //wpat(0xef); //end row number low byte
    WriteParameter( 0xEF );

    //widx(0x2c);       //enter write mode
    WriteIndex( 0x2C );

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );

    // dataport0 = sel_color[ i ];
//    c = sel_color[ i ];
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // dataport2 = sel_color[ i + 1 ];
//    c = sel_color[ i + 1 ];
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 14 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 15 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 16 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 17 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 18 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 19 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 0 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 2 ) );

    // dataport3 = sel_color[ i + 2 ];
//    c = sel_color[ i + 2 ];
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 3 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );

    // CS = 0;
    CLR_CS;

    // RS = 1;
    SET_RS;

    for( unsigned int i = 0; i < 240; i++ )
    {
        for( unsigned int j = 0; j < 320; j++ )
        {
            // clr_cs, set_rs moved outside of loop

            //set color
            // moved outside of loop

            // WR = 0;
            CLR_WR;

            // WR = 1;
            SET_WR;

            // set_cs, clr_rs moved after loop
        }
    }

    // CS = 1;
    SET_CS;

    // RS = 0;
    CLR_RS;
}

void FEHLCD::NextLine()
{
    _currentline++;
    if( _currentline > MAXLINES )
    {
        _currentline = 0;
        // TODO: clear screen
    }

    _currentchar = 0;
}
